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
# SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


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

# make a 'direction' class for xyz
#i.e, contract to 0, -1, or 1
def move_inc(p, v):
    INC = 0.05
    dp = (p[0] + v[0] * INC,
    p[1] + v[1] * INC,
    p[2] + v[2] * INC)
    return dp

def limit_dirs(p):
    LIMITS = (0.656982770038,
            -0.252598021641,
            0.5388609422173)

    def limit(pl):
        if pl[0] > pl[1]:
            return -1
        elif pl[0] < pl[1]:
            return 1
        else:
            return 0

    return map(limit, zip(p, LIMITS)) 
 	

def right_arm(pos):
    '''
    Create goal Pose and call ik move
    '''
    pose_right = Pose(
            position=Point(
                x=pos[0],
                y=pos[1],
                z=pos[2],
                ),
            orientation=Quaternion(
                x=0.367048116303,
                y=0.885911751787,
                z=-0.108908281936,
                w=0.261868353356,
                ),
            )

    ik_move('right', pose_right)

def ik_move(limb, pose):
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

        return resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return None

def make_move(msg, limb, speed):
        arm = baxter_interface.Limb(limb)
        lj = arm.joint_names()

        command = {}

        for i in range(0, len(msg.joints[0].name)):
            command[msg.joints[0].name[i]] = msg.joints[0].position[i]

        print command

        arm.set_joint_position_speed(speed)

        arm.move_to_joint_positions(command)


def recv_data(data):
    '''
    Take udp data and break into component messages
    '''
    
    # split on token
    list_data = data.split(',')
    
    # Convert next beat time
    list_data[0] = float(list_data[0])

    # Convert and normalize energy
    list_data[1] = float(list_data[1])
    list_data[1] = min(3000, list_data[1])
    list_data[1] /= 3000

    dict_data = {'beat': list_data[0], 'energy': list_data[1]}

    return dict_data

def create_socket():
    ''' 
    Create socket for incoming music data.
    This will change when music moves internally onto ros
    '''
    IP = "10.42.1.254"
    UDP_IP = "127.0.0.1"
    UDP_PORT = 54000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IP, UDP_PORT))
    return sock

def main():
    sock = create_socket()

    INIT_X=0.656982770038
    INIT_Y=-0.752598021641
    INIT_Z=0.5388609422173
    current_p = (INIT_X, INIT_Y, INIT_Z)
    directions = (0, 0, 0)

    # limit the amount of movement calls
    lim = 10
    j = 0
    while (j < lim):
        j += 1
        print "waiting at ", j
        # socket call
        data, addr = sock.recvfrom(1024)
        mdata = recv_data(data)

        # Wait till next beat
        ct = time.time()
        while (float(ct) < float(mdata['beat'])):
            ct = time.time()
            time.sleep(0.01)


        speed = mdata['energy']

        directions = limit_dirs(current_p)
	
    	current_p = move_inc(current_p, (0,1,0))		

        print "directions: ", directions
        print "current_p: ", current_p 

        resp = right_arm()
        if resp is not None:
            make_move(resp, 'right', speed)

if __name__ == '__main__':
    sys.exit(main())
