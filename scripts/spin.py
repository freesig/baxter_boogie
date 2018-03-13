#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def main():
	hz = 100;

  	pub = rospy.Publisher('/safebase/cmd_vel', Twist, queue_size=1)
	
