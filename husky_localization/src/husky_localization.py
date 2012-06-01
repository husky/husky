#!/usr/bin/python


import roslib; roslib.load_manifest('husky_localization')
import rospy
from threading import RLock

from clearpath_base.msg import Encoders, RotateRate
from clearpath_sensors.msg import GPSFix
from geometry_msgs.msg import Vector3, Twist, Pose2d

from numpy import matrix

import os, sys, getopt, math
from LatLongUTMconversion import LLtoUTM


# Kalman Filters
class Kalman():
    def __init__(self, wheel_sep):               
        self.wheel_sep = wheel_sep     
        self.init(0.0, 0.0, 0.0)

    def init(self, x, y, theta):       
        self.active = False
        self.xyrot = matrix([[x],
                             [y],
                             [theta]])
        self.last_vel_time = rospy.get_time()
        self.error_covariance =         matrix([[0.0, 0.0, 0.0], 
                                                [0.0, 0.0, 0.0],
                                                [0.0, 0.0, 0.0],])
        self.kalman_gain =              matrix([[0.0, 0.0, 0.0], 
                                                [0.0, 0.0, 0.0],
                                                [0.0, 0.0, 0.0]])
        self.process_error_coeff =      matrix([[0.0, 0.0, 0.0], 
                                                [0.0, 0.0, 0.0],
                                                [0.0, 0.0, 0.0]])
        self.identity_mat =             matrix([[1.0, 0.0, 0.0], 
                                                [0.0, 1.0, 0.0],
                                                [0.0, 0.0, 1.0]])         
        self.approx_vel = 0.0
        self.vel_dir = 0.0

    def predict_from_encoder(self, delL, delR, left_err, right_err):
        if self.active == False:
            return      

        new_time = rospy.get_time()
        delta_t = new_time - self.last_vel_time

        process_error = (math.fabs(left_err)+math.fabs(right_err))/2.0
        old_x, old_y, old_rot = self.xyrot

        rough_vel = ((delR+delL)/2.0)/delta_t
        slipvmin = 0.2
        slipvmax = 1.0
        slipmin = 1.0
        slipmax = 0.7
        straight_slip = max(slipmin, min(slipmax, ((slipmax-slipmin)/(slipvmax-slipvmin))*rough_vel))
        slipvmin = 0.2
        slipvmax = 1.0
        slipmin = 0.7
        slipmax = 0.3
        turn_slip = max(slipmin, min(slipmax, ((slipmax-slipmin)/(slipvmax-slipvmin))*rough_vel))

        avg_dist = ((delR+delL)/2.0)*straight_slip
        del_theta = ((delR-delL)/self.wheel_sep)*turn_slip
        avg_theta = old_rot + del_theta/2.0  

        self.xyrot[0] += avg_dist * math.cos(avg_theta)
        self.xyrot[1] += avg_dist * math.sin(avg_theta) 
        self.xyrot[2] += del_theta 

        self.vel_dir = (delL + delR) / 2.0        

        self.approx_vel = math.sqrt(pow(self.xyrot[0]-old_x,2) + pow(self.xyrot[1]-old_y,2))/delta_t
        self.last_vel_time = new_time

        k = avg_dist/(2.0*self.wheel_sep)
        heading_covar = (math.fabs(left_err)+math.fabs(right_err))/(self.wheel_sep)
        self.process_error_coeff[0] = [0.5*math.cos(avg_theta)-k*math.sin(avg_theta), 0.5*math.cos(avg_theta)+k*math.sin(avg_theta), 0]
        self.process_error_coeff[1] = [0.5*math.sin(avg_theta)+k*math.cos(avg_theta), 0.5*math.sin(avg_theta)-k*math.cos(avg_theta), 0]
        if process_error > 0.0:
            self.process_error_coeff[2] = [0, 0, heading_covar/process_error]    #[0, 0, 0.1]                                  
        else:
            self.process_error_coeff[2] = heading_covar
        self.error_covariance = self.error_covariance + (self.process_error_coeff*process_error)*self.process_error_coeff.T

    def wrapAngle(self, data):
        if data < 0.0:
            data = data + math.pi*2.0
        if data > math.pi*2.0:
            data = data - math.pi*2.0
        return data

    def measure_from_gps(self, x, y, gps_track, measure_error):
        if self.active == False:
            return

        if self.vel_dir < 0.0:
            gps_track = gps_track + math.pi

        # Make sure both angles are in the 0->360 range
        self.xyrot[2] = self.wrapAngle(self.xyrot[2])
        gps_track = self.wrapAngle(gps_track)

        # Make sure both angles are on the same side on 180 (expressed by adding or sub 2*pi)
        angleBet = 0.0;
        if gps_track < self.xyrot[2]:
            angleBet = self.xyrot[2] - gps_track
            if angleBet > math.pi:
                gps_track = gps_track + 2.0*math.pi
        else:
            angleBet = gps_track - self.xyrot[2]
            if angleBet > math.pi:
                gps_track = gps_track - 2.0*math.pi

        # Store
        gpsdata = matrix([[x],
                          [y],
                          [gps_track]])
               
        # Perform Update                             
        self.kalman_gain = self.error_covariance*((self.error_covariance + measure_error).I)
        self.xyrot = self.xyrot + self.kalman_gain*(gpsdata - self.xyrot)
        self.error_covariance = (self.identity_mat - self.kalman_gain)*self.error_covariance

        # Make sure angle stays in 0->360 range
        self.xyrot[2] = self.wrapAngle(self.xyrot[2])


class Encoders():
    def __init__(self, lock, xyrot, variance):
        self.odompublisher = odompublisher
        self.lock = lock
        self.last_update = rospy.Time.now() - rospy.Duration(5,0) # five seconds ago
        self.xyrot = xyrot
        self.last_encoder = None
        self.active = False

        # This is error in the travel measurement, due to
        # terrain, slip, tire pressure, etc.
        self.variance = variance

    def callback(self, data):
        with self.lock:                   
            if self.last_encoder:
                left_delta = data.encoders[0].travel - self.last_encoder.encoders[0].travel
                right_delta = data.encoders[1].travel - self.last_encoder.encoders[1].travel

                left_error = left_delta * self.variance
                right_error = right_delta * self.variance
                self.xyrot.predict_from_encoder(left_delta, right_delta, left_error, right_error)

            self.last_encoder = data
            self.last_update = rospy.Time.now()


class GPS():
    def __init__(self, lock, xyrot, variance):
        self.lock = lock
        self.last_update = rospy.Time.now() - rospy.Duration(5,0) # five seconds ago
        self.xyrot = xyrot
        self.active = False
        self.reference_ellipsoid = 23
        self.last_gps_data = None

        # The passed-in variance is variance in GPS position. The bottom-right term
        # concerning heading error is populated based on reported speed (in the callback).
        self.measure_error = matrix([[variance, 0.0, 0.0], 
                                     [0.0, variance, 0.0],
                                     [0.0, 0.0, 0.0]])

    def callback(self, data):
        with self.lock:
            try:
                # Convert GPS to XY coordinates.
                utm_zone, easting, northing = LLtoUTM(self.reference_ellipsoid, data.latitude, data.longitude)
                heading = (-data.track)*(math.pi/180.0) + (math.pi/2)
            except ValueError:
                # Bad data causes LL conversion to fail. Abort.
                return            

            # If we're moving slowly enough, the error on the heading makes it useless.
            if math.fabs(data.speed) < 0.4:
                self.measure_error[2,2] = 4.0*math.pi
            else:
                self.measure_error[2,2] = math.pi/8.0

            
            self.xyrot.measure_from_gps(easting, northing, heading, self.measure_error)    

            self.last_gps_data = (easting, northing, heading)
            self.last_update = rospy.Time.now()

    
# Main
class Localization():
    def __init__(self):
        rospy.init_node('localization_a100', anonymous=True)
        self.lock = RLock()      
        self.wheel_separation = float(rospy.get_param('wheel_separation', 0.5)) # m

        self.xyrot = Kalman(self.wheel_separation)

        self.encoders = EncoderDevice(self, self.lock, self.xyrot, 0.5) # 0.5 enc variance -> 50%
        self.gps = GPS(self, self.lock, self.xyrot, 0.5)

        rospy.Subscriber("data/encoders", Encoders, self.encoders.callback)
        rospy.Subscriber("fix", GPSFix, self.gps_callback)

        #self.odometry_pub = rospy.Publisher('ekf/odom', Odometry, latch=True)
        self.pose_pub = rospy.Publisher('pose', Pose2D, latch=True)

        self.UPDATE_HZ = 20
        self.rate = rospy.Rate(self.UPDATE_HZ)

    def is_gps_active(self):
        with self.lock:
            if (rospy.Time.now() - self.gps.last_update) < rospy.Duration(2,0):
                return True
            else:
                return False

    def disable_KF(self):
        with self.lock:
            self.xyrot.active = False  
            self.encoders.active = False
            self.gps.active = False

    def enable_KF(self):
        with self.lock:
            self.xyrot.active = True  
            self.encoders.active = True
            self.gps.active = True
    
    def initialize_from_gps(self):
        with self.lock:
            x, y, heading = self.gps.last_gps_data
            self.xyrot.init(x, y, heading)

    def run(self):
        rospy.sleep(0.5)
        self.lps_active = False
        self.gps_active = False
        while not rospy.is_shutdown():
            self.rate.sleep()

            # check if lps is recent
            if self.is_gps_active() == True:
                if self.gps_active == False:
                    #initialize from gps
                    self.initialize_from_gps()
                    self.enable_KF()
                self.gps_active = True
                self.publish_odom_message_from_odom()
            else:
                # nothing recent, kill it all!
                if self.gps_active == True:
                    self.disable_KF()
                self.gps_active = False
                self.lps_active = False
            

Localization().run()
