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
