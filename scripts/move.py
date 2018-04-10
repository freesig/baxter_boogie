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

import _thread
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

def right_arm():
    pose_right_init = Pose(
            position=Point(
                x=0.656982770038,
                y=-0.752598021641,
                z=0.5388609422173,
                ),
            orientation=Quaternion(
                x=0.367048116303,
                y=0.885911751787,
                z=-0.108908281936,
                w=0.261868353356,
                ),
            )

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
                ),
            )
    '''

    pose_left_init = Pose(
            position=Point(
                x=0.656982770038,
                y=0.052598021641,
                z=0.5388609422173,
                ),
            orientation=Quaternion(
                x=0.367048116303,
                y=0.885911751787,
                z=-0.108908281936,
                w=0.261868353356,
                ),
            )

    pose_left_goal = Pose(
            position=Point(
                x=0.656982770038,
                y=0.552598021641,
                z=0.5388609422173,
                ),
            orientation=Quaternion(
                x=0.367048116303,
                y=0.885911751787,
                z=-0.108908281936,
                w=0.261868353356,
                ),
            )


    limit = 10;
    i = 0;

    while (i < limit):
        i += 1;
        ik_test('right', pose_right_init)
        ik_test('left', pose_left_init)
        time.sleep(3);
        ik_test('right', pose_right_goal)
        ik_test('left', pose_left_goal)
    '''
    limit = 10;
    i = 0;

    while (i < limit):
        i += 1;
        ik_test('right', pose_right_init)
        time.sleep(3);
        ik_test('right', pose_right_goal)



def left_arm():
    pose_left_init = Pose(
            position=Point(
                x=0.656982770038,
                y=0.052598021641,
                z=0.5388609422173,
                ),
            orientation=Quaternion(
                x=0.367048116303,
                y=0.885911751787,
                z=-0.108908281936,
                w=0.261868353356,
                ),
            )

    pose_left_goal = Pose(
            position=Point(
                x=0.656982770038,
                y=0.552598021641,
                z=0.5388609422173,
                ),
            orientation=Quaternion(
                x=0.367048116303,
                y=0.885911751787,
                z=-0.108908281936,
                w=0.261868353356,
                ),
            )

    limit = 10;
    i = 0;

    while (i < limit):
        i += 1;
        ik_test('left', pose_right_init)
        time.sleep(3);
        ik_test('left', pose_right_goal)

def ik_test(limb, pose):
    rospy.init_node("rsdk_ik_service_client")
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

        arm.set_joint_position_speed(1.0);





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
    #return right_arm()
    print("Ready to Go!")
    _thread.start_new_thread(right_arm(),())
    _thread.start_new_thread(left_arm(),())

    '''
    limit = 10
    i = 1

    while (i < limit):
        _thread.start_new_thread(ik_test,('right',pose_right_init))
        _thread.start_new_thread(ik_test,('left',pose_left_init))
        sleep(3)
        _thread.start_new_thread(ik_test,('right',pose_right_goal))
        _thread.start_new_thread(ik_test,('left',pose_left_goal))
    '''



if __name__ == '__main__':
    sys.exit(main())
