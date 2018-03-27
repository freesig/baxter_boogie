#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from geometry_msgs.msg import Twist

from beginner_tutorials.msg import music_keys

base_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
direction = 1
beat_waiting = False;

import thread
import time

def beat(timer, num):
    global direction
    global beat_waiting
    beat_waiting = True;
    time.sleep(timer);
    direction = direction * -1;
    beat_waiting = False;

def send_move(publisher, music_keys):
    msg = Twist()
    msg.linear.x = music_keys.volume*direction;
    msg.angular.z = music_keys.volume;
    publisher.publish(msg);
    if not beat_waiting:
        thread.start_new_thread(beat, (music_keys.beat, 0));

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard vol: %s", data.volume)
    send_move(base_pub, data);

def main():

    hz = 10;

    rospy.init_node('test', anonymous=True)
    rospy.Subscriber("chatter", music_keys, callback)
    rate = rospy.Rate(hz) # 10hz

    frame_time_ms = 1000 / hz;
    duration_ms = 5000; # ms
    elapsed = 0;

    while not rospy.is_shutdown() and elapsed < duration_ms:
        elapsed += frame_time_ms;
        #send_move(base_pub, [1.0, 1.0]);
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
