#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import time

from geometry_msgs.msg import Twist

#from beginner_tutorials.msg import music_keys

import socket
IP = "10.42.1.254"
UDP_IP = "127.0.0.1"
UDP_PORT = 65000

#base_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
base_pub = rospy.Publisher('/safebase/cmd_vel', Twist, queue_size=1)
direction = 1
beat_waiting = False;

import thread
import time

DEFAULT_SPEED = 0.0
DEFAULT_SPIN = 1.0

beat_num = 0

def beat(timer, num):
    global direction
    global beat_waiting
    global beat_num
    beat_waiting = True;
    time.sleep(timer);
    if (beat_num % 2 == 0):
        direction = direction * -1;
    beat_num += 1
    beat_waiting = False;

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard vol: %s", data.volume)
    send_move(base_pub, data);

def recv_data(data, publisher):
    data = float(data);
    print (time.time());
    print (data);
    time_until = data - time.time();
    print time_until;
    if not beat_waiting:
        thread.start_new_thread(beat, (DEFAULT_SPEED, 0));

def do_publish(publisher, nil):
    hz = 10;
    frame_time_ms = 1000 / hz;
    duration_ms = 20000; # ms
    elapsed = 0;
    rate = rospy.Rate(hz) # 10hz
    while not rospy.is_shutdown() and elapsed < duration_ms:
        msg = Twist()
        msg.linear.x = DEFAULT_SPEED;
        print msg.linear.x;
        msg.angular.z = DEFAULT_SPIN*direction;
        publisher.publish(msg);
        elapsed += frame_time_ms;
        rate.sleep()

def main():

    hz = 10;

    rospy.init_node('test', anonymous=True)
    #rospy.Subscriber("chatter", music_keys, callback)
    rate = rospy.Rate(hz) # 10hz

    frame_time_ms = 1000 / hz;
    duration_ms = 2000; # ms
    elapsed = 0;

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IP, UDP_PORT))

    thread.start_new_thread(do_publish, (base_pub, 0));

    while True:
        print "waiting"
        data, addr = sock.recvfrom(1024);
        recv_data(data, base_pub);


        #rate.sleep()
        #send_move(base_pub, [1.0, 1.0]);

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
