#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def spin():
	pub = rospy.Publisher('/safebase/cmd_vel', Twist, queue_size=1)

	rospy.init_node('spin_test');

	msg = Twist();
	msg.angular.z = 0.1;
	msg.linear.x = 0.0;
	msg.linear.y = 0.0;

	hz = 10;
	frame_time_ms = 1000 / hz;
	duration_ms = 2000;
	elapsed = 0;	

	count = 200;
	i = 0;

	while not rospy.is_shutdown() and i < count:
		i += 1;
		elapsed += frame_time_ms;
		msg = Twist();
		msg.angular.z = 0.2;
		msg.linear.x = 0.1;
		msg.linear.y = 0.0;
		print("sending: ");
		print msg.linear.x;
		pub.publish(msg);
		rospy.sleep(0.1);

if __name__ == '__main__':
	spin()
