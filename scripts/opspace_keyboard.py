#!/usr/bin/env python
# 2015 Juxi Leitner
# building on the Rethink Robotics Baxter Example:
# RSDK Joint Position Example: Keyboard Control

import argparse
import rospy

import baxter_interface
import baxter_external_devices

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

def move_end_effector(limb_name, axis, delta):
    #print "right:" + str(right.endpoint_pose()['position'][0])
    limb_name = 'right'
    limb = baxter_interface.Limb(limb_name)
    current_pose = limb.endpoint_pose()
    
    ns = "ExternalTools/" + limb_name +"/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()

    # build the message header
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    # build the new position
    try:
        pos = Point( x = current_pose['position'][0],
                     y = current_pose['position'][1],
                     z = current_pose['position'][2] )
        if axis == 'X': pos.x += delta
        if axis == 'Y': pos.y += delta
        if axis == 'Z': pos.z += delta
    except KeyError, e:
	print "argh"
	return 1

    pose = PoseStamped(header=hdr, pose=Pose( position = pos, orientation = current_pose['orientation'] ) )
    ikreq.pose_stamp.append(pose)
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    if (resp.isValid[0]):
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        # print limb_joints
        limb.set_joint_positions(limb_joints)
        rospy.sleep(0.01)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
    return 0


def map_keyboard():
    bindings = {
    #   key: (function, args, description)
        'a': (move_end_effector, ['right', 'Y',  0.1], "Y increase"),
        'd': (move_end_effector, ['right', 'Y', -0.1], "Y decrease"),
        'w': (move_end_effector, ['right', 'X',  0.1], "x increase"),
        's': (move_end_effector, ['right', 'X', -0.1], "X decrease"),
     }
    done = False
    print("Controlling robot. Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c in bindings:
                cmd = bindings[c]
                #expand binding to something like "set_j(right, 's0', 0.1)"
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))


def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
#    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
#                                     description=main.__doc__)
#    parser.add_argument()
#    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("opspace_keyboard")

    def clean_shutdown():
        print("\nExiting example...")
    rospy.on_shutdown(clean_shutdown)

    map_keyboard()
    print("Done.")


if __name__ == '__main__':
    main()
