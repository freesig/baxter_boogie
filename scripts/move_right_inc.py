#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Example
"""

import thread
import threading

import time

import argparse
import struct
import sys

import rospy

from geometry_msgs.msg import (
        PoseStamped,
        Pose,
        Point,
        Quaternion,
        )
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
        SolvePositionIK,
        SolvePositionIKRequest,
        )

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

import socket
IP = "10.42.1.254"
UDP_IP = "127.0.0.1"
UDP_PORT = 54000

SPEED = 1.0;

at_init = True;

INIT_X=0.656982770038
INIT_Y=-0.752598021641
INIT_Z=0.5388609422173

LIMIT_X=0.656982770038
LIMIT_Y=-0.252598021641
LIMIT_Z=0.5388609422173

current_x = INIT_X
current_y = INIT_Y
current_z = INIT_Z

INC = 0.05
# make a 'direction' class for xyz
#i.e, contract to 0, -1, or 1
def move_inc(x,y,z):
	current_x += x*INC;
	current_y += y*INC;
	current_z += z*INC;
 	

def right_arm():
    pose_right_init = Pose(
            position=Point(
                x=current_x,
                y=current_y,
                z=current_z,
                ),
            orientation=Quaternion(
                x=0.367048116303,
                y=0.885911751787,
                z=-0.108908281936,
                w=0.261868353356,
                ),
            )
"""

    pose_right_goal = Pose(
            position=Point(
                x=0.656982770038,
                y=-0.252598021641,
                z=0.5388609422173,
                ),
            orientation=Quaternion(
                x=0.367048116303,
                y=0.885911751787,
                z=-0.108908281936,
                w=0.261868353356,
                )

            )

    limit = 1;
    i = 0;

    global at_init;

    while (i < limit):
        i += 1;
        if (at_init):
            ik_test('right', pose_right_init)
        else: 
            ik_test('right', pose_right_goal)
        time.sleep(1);

    at_init = not at_init;
"""
    ik_test('right', pose_right_init)

def ik_test(limb, pose):
    rospy.init_node("move_right")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ps = PoseStamped(
            header=hdr,
            pose=pose,
            )

    ikreq.pose_stamp.append(ps)
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
            resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                ikreq.SEED_USER: 'User Provided Seed',
                ikreq.SEED_CURRENT: 'Current Joint Angles',
                ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp



        arm = baxter_interface.Limb(limb)
        lj = arm.joint_names()

        #current_position = left.joint_angle(lj[3])
        #joint_command = {lj[3]: current_position - delta}

        command = {};

        for i in range(0, len(resp.joints[0].name)):
            command[resp.joints[0].name[i]] = resp.joints[0].position[i];

        print command;

        arm.move_to_joint_positions(command)

        global SPEED;

        arm.set_joint_position_speed(SPEED);





    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return 0


def main():
    """RSDK Inverse Kinematics Example

    A simple example of using the Rethink Inverse Kinematics
    Service which returns the joint angles and validity for
    a requested Cartesian Pose.

    Run this example, passing the *limb* to test, and the
    example will call the Service with a sample Cartesian
    Pose, pre-defined in the example code, printing the
    response of whether a valid joint solution was found,
    and if so, the corresponding joint angles.
    """

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IP, UDP_PORT))

    # run right arm in a thread

    lim = 10;
    j = 0;

    global SPEED;

    while (j < lim):
        j += 1;
        print "waiting at ", j
        data, addr = sock.recvfrom(1024);
        mdata = recv_data(data, 1);

        ct = time.time()

        while (float(ct) < float(mdata[0])):

            ct = time.time();
            time.sleep(0.01);


        SPEED = mdata[1];

		x_dir = 0;
		y_dir = 0;
		z_dir = 0;
				

		if (current_x > LIMIT_X):
			x_dir = -1
		elif(current_x < LIMIT_X): 
			x_dir = 1

		if (current_y > LIMIT_Y):
			y_dir = -1
		elif(current_y < LIMIT_Y): 
			y_dir = 1
	
		
		if (current_z > LIMIT_Z):
			z_dir = -1
		elif(current_z < LIMIT_Z): 
			z_dir = 1
	
		
		move_inc(x_dir,y_dir,z_dir)		

        right_arm();


def recv_data(data, p):

    list_data = data.split(',');

    print list_data[1];
    list_data[1] = float(list_data[1]);

    list_data[1] = min(3000, list_data[1]);
    list_data[1] /= 3000;

    list_data[0] = float(list_data[0]);
    return list_data;

if __name__ == '__main__':
    sys.exit(main())