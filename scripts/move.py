import socket
import time
import Vectors
import struct

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

def ik_move(limb, pose):
    rospy.init_node("move_" + limb)
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

# make a 'direction' class for xyz
#i.e, contract to 0, -1, or 1
def move_inc(p, v, inc_v):
    inc = inc_v.y()
    return map(lambda pv: pv[0] + pv[1] * inc, zip(p, v))

def limit_dirs(p, bound):
    limits = (bound.x(), bound.y(), bound.z())

    def limit(pl):
        if pl[0] > pl[1]:
            return -1
        elif pl[0] < pl[1]:
            return 1
        else:
            return 0

    return map(limit, zip(p, limits)) 

def decide_increment(data): 
    #TODO - decide based on data
    INC = Vectors.V4D(0.00, 0.05, 0.00, 0);
    return INC;

def run(sock, arm, pose_func, edges):
    current_p = (edges['init'].x(), edges['init'].y(), edges['init'].z())
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

        directions = limit_dirs(current_p, edges['bound'])
        
        increment = decide_increment(mdata); 
        current_p = move_inc(current_p, directions, increment)		

        print "directions: ", directions
        print "current_p: ", current_p 
        pose_func(current_p)

        resp = ik_move(arm, pose_func(current_p))
        if resp is not None:
            make_move(resp, arm, speed)
