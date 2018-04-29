import socket
import time
import Vectors
import struct
import threading
import Queue
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

direction = 1

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
        #print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" % (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

        return resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return None

def make_move(msg, limb, speed):
    SPEED_SCALE = 4
    speed = speed * SPEED_SCALE
    arm = baxter_interface.Limb(limb)
    lj = arm.joint_names()

    command = {}
    for i in range(0, len(msg.joints[0].name)):
        command[msg.joints[0].name[i]] = msg.joints[0].position[i]
    

    arm.set_joint_position_speed(speed)

    arm.set_joint_positions(command)


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

def decide_increment(data, inc): 
    #TODO - decide based on data
    inc = Vectors.V4D(inc.x(), -1 * inc.y(), inc.z(), inc.w())
    return inc;

def clamp(p, inner, outer):
    if p.x() > outer.x():
        p = Vectors.V4D(outer.x(), p.y(), p.z(), p.w())
    elif p.x() < inner.x():
        p = Vectors.V4D(inner.x(), p.y(), p.z(), p.w())
    if p.y() > outer.y():
        p = Vectors.V4D(p.x(), outer.y(), p.z(), p.w())
    elif p.y() < inner.y():
        p = Vectors.V4D(p.x(), inner.y(), p.z(), p.w())
    if p.z() > outer.z():
        p = Vectors.V4D(p.x(), p.y(), outer.z(), p.w())
    elif p.z() < inner.z():
        p = Vectors.V4D(p.x(), p.y(), inner.z(), p.w())
    if p.w() > outer.w():
        p = Vectors.V4D(p.x(), p.y(), p.z(), outer.w())
    elif p.w() < inner.w():
        p = Vectors.V4D(p.x(), p.y(), p.z(), inner.w())

    return p





def run(sock, arm, pose_func, edges):
    current_p = edges['init']
    i_bound = edges['init']
    o_bound = edges['bound']


    channel = Queue.Queue()

    # initial values
    increment = Vectors.V4D(0.00, 0.05, 0.00, 0);
    speed = 0.5

    # limit the amount of movement calls
    lim = 1000
    j = 0
	mutex = threading.Lock()

    while (j < lim):
        j += 1
        
        # socket call
        try:
            data, addr = sock.recvfrom(1024)
            mdata = recv_data(data)
            speed = mdata['energy']
            def swap_inc(data, inc):
                while time.time() < data['beat']:
                    time.sleep(0.01)
                global direction
				mutex.acquire()
                direction *= -1
				mutex.release()
                inc = Vectors.V4D(inc.x(), direction * inc.y(), inc.z(), inc.w())
                print "sent"
                channel.put(inc)
			t = threading.Thread(target = swap_inc, args = (mdata, increment))
			t.start()
			t.join() 

        except socket.error, e:
           None 
        
        try:
            increment = channel.get(False)
            print "got"
            increment.display()
        except Queue.Empty:
            None 
        
        current_p = current_p + increment

        current_p = clamp(current_p, i_bound, o_bound)

        pose_func(current_p)

        resp = ik_move(arm, pose_func(current_p))

        if resp is not None:
            make_move(resp, arm, speed)

        time.sleep(0.1)
