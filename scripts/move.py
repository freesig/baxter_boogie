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

def make_move(msg, limb, speed, last_resp):
    SPEED_SCALE = 4
    speed = speed * SPEED_SCALE
    arm = baxter_interface.Limb(limb)
    lj = arm.joint_names()

    if last_resp is None:
        return 0

    last_p = last_resp.joints[0].position
    command = {}
    for i in range(0, len(msg.joints[0].name)):
        command[msg.joints[0].name[i]] = msg.joints[0].position[i]
        print "pos: ", msg.joints[0].position[i]
        print "last pos: ", last_p[i] 
    
    print command

    arm.set_joint_position_speed(speed)

    print "before move loop: ", time.time()
    arm.set_joint_positions(command)
    time.sleep(0.1)
    print "after move loop: ", time.time()


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
def move_inc(p, inc):
    inc = (inc.x(), inc.y(), inc.z())
    return map(lambda pi: pi[0] + pi[1], zip(p, inc))

#def limit_dirs(p, inner_bound, outer_bound, inc):


    #def limit(pl):
        #if pl[0] > pl[1]:
            #return pl[3] * -1
        #else:
            #return pl[3]

    #return map(limit, zip(p, limits, inner_bound, directions)) 

def decide_increment(data): 
    #TODO - decide based on data
    INC = Vectors.V4D(0.00, 0.05, 0.00, 0);
    return INC;

def clamp(p, inner, outer, inc):
    inc = (inc.x(), inc.y(), inc.z())
    def c(x):
        if x[0] < x[1]:
            return (x[1], x[3] * -1)
        if x[0] > x[2]:
            return (x[2], x[3] * -1)
        else:
            return (x[0], x[3])
    return map(c, zip(p, inner, outer, inc))

def run(sock, arm, pose_func, edges):
    current_p = (edges['init'].x(), edges['init'].y(), edges['init'].z())
    init_p = (edges['init'].x(), edges['init'].y(), edges['init'].z())
    bound = (edges['bound'].x(), edges['bound'].y(), edges['bound'].z())
    directions = (0, 1, 0)

    # limit the amount of movement calls
    lim = 100
    j = 0

    increment = Vectors.V4D(0.00, 0.05, 0.00, 0);
    last_resp = None
    speed = 0.5
    while (j < lim):
        print "time start loop: ", time.time()
        j += 1
        print "waiting at ", j
        # Wait till next beat
        # socket call
        try:
            data, addr = sock.recvfrom(1024)
            mdata = recv_data(data)
            speed = mdata['energy']
        except socket.error, e:
            print "missed"

        


        '''
        ct = time.time()
        while (float(ct) < float(mdata['beat'])):
            ct = time.time()
            time.sleep(0.01)
        '''



        #directions = limit_dirs(current_p, edges['bound'], directions)
        #increment = decide_increment(mdata); 
        
        current_p = move_inc(current_p, increment)

        something = clamp(current_p, init_p, bound, increment)
        current_p = map(lambda x: x[0], something)
        inc = map(lambda x: x[1], something)
        increment = Vectors.V4D(inc[0], inc[1], inc[2], 0)


        print "directions: ", directions
        print "current_p: ", current_p 
        pose_func(current_p)

        print "time before ik: ", time.time()

        resp = ik_move(arm, pose_func(current_p))

        print "time after ik: ", time.time()
        if resp is not None:
            make_move(resp, arm, speed, last_resp)

        print "time end loop: ", time.time()
        last_resp = resp
