#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

################################################################################

LIN_VEL_LIMIT = 1.0 # m/s
ANG_VEL_LIMIT = 2.0 # rads
LIN_VEL_STEP  = 0.1
ANG_VEL_STEP  = 0.1

info = """
----------------------------------------
Husky Robot Keyboard Teleoperation Panel
----------------------------------------
                 W
             A   S   D
                 X
W/S : Increase/decrease linear velocity
D/A : Increase/decrease angular velocity
X   : Emergency brake
Press CTRL+C to quit
NOTE: Press keys within this terminal
----------------------------------------
"""

error = """
ERROR: Communication failed!
"""

def get_key():
    if os.name == 'nt':
      return msvcrt.getch()
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input
    return input

def bound_lin_vel(lin_vel_cmd):
    lin_vel_cmd = constrain(lin_vel_cmd, -LIN_VEL_LIMIT, LIN_VEL_LIMIT)
    return lin_vel_cmd

def bound_ang_vel(ang_vel_cmd):
    ang_vel_cmd = constrain(ang_vel_cmd, -ANG_VEL_LIMIT, ANG_VEL_LIMIT)
    return ang_vel_cmd

def generate_cmd_vel_msg(lin_vel_cmd, ang_vel_cmd):
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x  = lin_vel_cmd
    cmd_vel_msg.linear.y  = 0.0
    cmd_vel_msg.linear.z  = 0.0
    cmd_vel_msg.angular.x = 0.0
    cmd_vel_msg.angular.y = 0.0
    cmd_vel_msg.angular.z = ang_vel_cmd
    return cmd_vel_msg
