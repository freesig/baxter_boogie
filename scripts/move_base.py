#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import time

from geometry_msgs.msg import Twist

import socket

#base_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
base_pub = rospy.Publisher('/safebase/cmd_vel', Twist, queue_size=1)
direction = 1
beat_waiting = False;

import thread
import time

DEFAULT_SPEED = 0.0
DEFAULT_SPIN = 0.8

spin_speed = DEFAULT_SPEED;

beat_num = 0

def beat(timer, num):
    global direction
    global beat_waiting
    global beat_num
    beat_waiting = True;
    time.sleep(timer);
    #if (beat_num % 2 == 0):
    direction = direction * -1;
    #beat_num += 1
    #beat_waiting = False;

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard vol: %s", data.volume)
    send_move(base_pub, data);

def recv_data(data, publisher):

    global spin_speed;

    list_data = data.split(',');

    list_data[1] = float(list_data[1]);

    list_data[1] = min(1700, list_data[1]);
    list_data[1] /= 1700;

    print list_data[1];
    #list_data[1] *= 0.5;

    spin_speed = list_data[1];

    list_data[0] = float(list_data[0]);
    print (time.time());
    #print (data);
    time_until = list_data[0] - time.time();
    print time_until;
    #if not beat_waiting:
    thread.start_new_thread(beat, (time_until, 0));

def do_publish(publisher, nil):
    hz = 10;
    frame_time_ms = 1000 / hz;
    duration_ms = 60000; # ms
    elapsed = 0;
    rate = rospy.Rate(hz) # 10hz
    while not rospy.is_shutdown() and elapsed < duration_ms:
        msg = Twist()
        msg.linear.x = DEFAULT_SPEED;
        #print msg.linear.x;

        #print ("spin speed", spin_speed);

        msg.angular.z = spin_speed*direction*2;
        publisher.publish(msg);
        elapsed += frame_time_ms;
        rate.sleep()

def create_socket():
    IP = "10.42.1.254"
    UDP_IP = "127.0.0.1"
    UDP_PORT = 52000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IP, UDP_PORT))


def main():
    sock = create_socket()

    hz = 10;

    rospy.init_node('test', anonymous=True)

    frame_time_ms = 1000 / hz;
    duration_ms = 2000; # ms
    elapsed = 0;

    thread.start_new_thread(do_publish, (base_pub, 0));

    print "comment base"

    while True:
        print "waiting"
        data, addr = sock.recvfrom(1024);
        recv_data(data, base_pub);


        #rate.sleep()
        #send_move(base_pub, [1.0, 1.0]);

    rospy.spin()
    socket.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
