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


import socket

import move

import Vectors

def right_arm(pos):
    '''
    Create goal Pose and call ik move
    '''
    pose_right = Pose(
            position=Point(
                x=pos.x(),
                y=pos.y(),
                z=pos.z(),
                ),
            orientation=Quaternion(
                x=0.367048116303,
                y=0.885911751787,
                z=-0.108908281936,
                w=0.261868353356,
                ),
            )
    return pose_right



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
    sock.setblocking(0)
    return sock

def main():
    init_pos = Vectors.V4D(0.656982770038,
            -0.752598021641,
            0.0388609422173, 0)
    bound = Vectors.V4D(0.656982770038,
            -0.252598021641,
            0.5388609422173, 0)
    edges = {'init': init_pos, 'bound': bound};
    sock = create_socket()
    try:
        move.run(sock, 'right', right_arm, edges)
    except KeyboardInterrupt:
        sock.close()


if __name__ == '__main__':
    sys.exit(main())
