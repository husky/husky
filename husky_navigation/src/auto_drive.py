#! /usr/bin/env python
import roslib; roslib.load_manifest('husky_navigation')

import rospy
import math
import numpy
import geometry_msgs

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion


class AutoDrive(object):
    def __init__(self):
        rospy.init_node('auto_drive')
        self.enable = False
        self.watchdog = False
        self.curr_odom = PoseWithCovarianceStamped()
        self.first_odom = PoseWithCovarianceStamped()
        self.received_odom = False
        self.received_first_odom = False
        self.command_pub = rospy.Publisher('husky/cmd_vel',Twist)

        joy_sub = rospy.Subscriber('teleop/joy',Joy,self.joy_callback)
        odom_sub = rospy.Subscriber('robot_pose_ekf/odom',PoseWithCovarianceStamped,self.odom_callback)
        rospy.Timer(rospy.Duration(0.05), self.output_latest_command)
        rospy.Timer(rospy.Duration(0.5), self.odom_watchdog)   

        #variables used for calculating trajectory controller
        self.last_theta_meas = 0
        self.curr_theta = 0
        self.last_curr_theta = 0
        self.curr_wrap_counter = 0
        self.r = rospy.get_param('~radius',3)
        self.ky = 0.5
        self.kc = 0.1
        self.reqd_fwd = rospy.get_param('~fwd_vel',0.5)

    def joy_callback(self,data):
        if data.buttons[3] == 1:
            self.enable = True
            rospy.loginfo('Autonomous On')
        else:
            rospy.loginfo('Autonomous Off')
            self.enable = False

    def odom_callback(self,data):
        if self.received_first_odom:
            self.received_odom = True
            self.curr_odom = data
        else:
            self.first_odom = data
            self.received_first_odom = True

    def odom_watchdog(self,event):
        if self.received_odom:
            self.watchdog = True
            self.received_odom = False
        else:
            watchdog = False
			

    def output_latest_command(self,event):
        if self.enable and self.watchdog:
            latest_twist = Twist()
            x = self.curr_odom.pose.pose.position.x
            y = self.curr_odom.pose.pose.position.y
            quat = self.curr_odom.pose.pose.orientation
            euler = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
            yaw = euler[2]

            rospy.loginfo("X:%f,Y:%f,yaw:%f",x,y,yaw)

            self.curr_theta = math.atan2(y,x);

            goal_theta = self.curr_theta + (math.pi)/2

            e_ct = -self.r + math.fabs(math.sqrt(x*x + y*y))
            e_yaw = goal_theta - yaw
            e_yaw = math.sin(e_yaw)
            e_yaw = math.asin(e_yaw)

            reqd_turn = self.ky * (e_yaw) + math.atan2(self.kc*e_ct,self.reqd_fwd)
            rospy.loginfo("ct:%f,ey:%f",math.atan2(self.kc*e_ct,self.reqd_fwd),self.ky*e_yaw)
            rospy.loginfo("control_output: %f",reqd_turn)
            latest_twist.linear.x = self.reqd_fwd;
            latest_twist.angular.z = reqd_turn;

            self.command_pub.publish(latest_twist)
    	elif self.enable:
    		rospy.loginfo("No new position data :(");	

if __name__ == '__main__':
    obj = AutoDrive()
    rospy.loginfo("Autonomous Drive Node On: Waiting for auto 'Y' button")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
