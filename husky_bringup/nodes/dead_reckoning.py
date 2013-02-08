#!/usr/bin/env python
import roslib; roslib.load_manifest('husky_bringup')
import roslib.rosenv
import rospy
import PyKDL
import copy

from math import sin,cos
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from clearpath_base.msg import Encoders

# Dynamic Reconfigure
import dynamic_reconfigure.server
from husky_bringup.cfg import HuskyConfig

ODOM_POSE_COVAR_MOTION = [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e6]
ODOM_POSE_COVAR_NOMOVE = [1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9]

ODOM_TWIST_COVAR_MOTION = [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e6]
ODOM_TWIST_COVAR_NOMOVE = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]


class DeadReckoning(object):
    def __init__(self):
        rospy.init_node('dead_reckoning')
        # Parameters
        self.width = rospy.get_param('~width',0.5)
        self.gyro_scale = rospy.get_param('~gyro_scale_correction',1.0)

        # Set up publishers/subscribers
        self.pub_imu = rospy.Publisher('imu_data',Imu)
        self.pub_enc_odom = rospy.Publisher('encoder',Odometry);
        rospy.Subscriber('imu/data', Imu, self.HandleIMU)
        rospy.Subscriber('husky/data/encoders', Encoders, self.HandleEncoder)

        # Initialize odometry message
        self.odom = Odometry()
        self.odom.header.frame_id = "odom_combined"
        self.odom.child_frame_id = "base_footprint" 
        self.last_encoder = []

        dynamic_reconfigure.server.Server(HuskyConfig, self.Reconfigure)

    def Reconfigure(self, config, level):
        # Handler for dynamic_reconfigure
        self.gyro_scale = config['gyro_scale_correction']
        return config

    def HandleIMU(self,data):
	# Correct IMU data
	# Right now, gyro scale only
	# TODO: Evaluate necessity of adding drift correction 
        imu_corr = copy.deepcopy(data)
        imu_corr.header.frame_id = "base_link"
        imu_corr.angular_velocity.z = data.angular_velocity.z * self.gyro_scale
        self.pub_imu.publish(imu_corr)
   
    def HandleEncoder(self,data):
        # Initialize encoder state
        if not self.last_encoder:
            self.last_encoder = data
            return
        # Calculate deltas
        dr = ((data.encoders[0].travel - self.last_encoder.encoders[0].travel) +
              (data.encoders[1].travel - self.last_encoder.encoders[1].travel)) / 2;
        da = ((data.encoders[1].travel - self.last_encoder.encoders[1].travel) - 
              (data.encoders[0].travel - self.last_encoder.encoders[0].travel)) / self.width;
        ddr = (data.encoders[0].speed + data.encoders[1].speed)/2;
        dda = (data.encoders[1].speed - data.encoders[0].speed)/self.width;
        self.last_encoder = data

        # Update data
        o = self.odom.pose.pose.orientation
        cur_heading = PyKDL.Rotation.Quaternion(o.x,o.y,o.z,o.w).GetEulerZYX()
        self.odom.pose.pose.position.x += dr * cos(cur_heading[0])
        self.odom.pose.pose.position.y += dr * sin(cur_heading[0]) 
        quat = PyKDL.Rotation.RotZ(cur_heading[0] + da).GetQuaternion()
        self.odom.pose.pose.orientation = Quaternion(quat[0],quat[1],quat[2],quat[3])
        self.odom.twist.twist.linear.x = ddr
        self.odom.twist.twist.angular.z = dda
        
        self.odom.header.stamp = rospy.Time.now()

        # If our wheels aren't moving, we're likely not moving at all.
        # Adjust covariance appropriately
        if data.encoders[0].speed == 0 and data.encoders[1].speed == 0:
            self.odom.pose.covariance = ODOM_POSE_COVAR_NOMOVE
            self.odom.twist.covariance = ODOM_TWIST_COVAR_NOMOVE
        else:
            self.odom.pose.covariance = ODOM_POSE_COVAR_MOTION
            self.odom.twist.covariance = ODOM_TWIST_COVAR_MOTION

        self.pub_enc_odom.publish(self.odom)

if __name__ == "__main__":
    obj = DeadReckoning()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
