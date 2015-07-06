#!/usr/bin/python
##########################################################################
# Simple IK moving of the Baxter arm
# copyleft: Juxi Leitner <j.leitner@roboticvision.org>
# Australian Centre for Robotic Vision (ACRV)
# Queensland University of Techonology (QUT)
#
# Coded for the QUT Robotics and Autonomous Systems Winter
# School, 2015, Brisbane
# http://Juxi.net/WinterSchool/2015/
#
# modified from the RethinkRobotics SDK tutorials, work at QUT, and by
# Allen Liang <h2.liang@student.qut.edu.au>
#
##########################################################################

import argparse
import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION
from std_msgs.msg import Header
 
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

class MoveArm(object):
    def __init__(self):
        self.initialize_arm()

    def initialize_arm(self):
        # initialize interfaces
        print("Getting robot state... ")
        self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self.init_state = self.rs.state().enabled
        self.limb = baxter_interface.Limb('right')
        rospy.on_shutdown(self.clean_shutdown)

        limb_name = "right"
        self.ik_name = "ExternalTools/" + limb_name +"/PositionKinematicsNode/IKService"
        self.iksvc = rospy.ServiceProxy(self.ik_name, SolvePositionIK)
        print "Initialized!"


    def clean_shutdown(self):
        if not self.init_state:
            print("Disabling robot...")
            self.rs.disable()
        print("Exiting example.")

    def current_pose(self):
        return self.limb.endpoint_pose()

    def to_XYZ(self, position):
        current_pose = self.limb.endpoint_pose()

        # build the message header
        self.ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        # build the new position
        #    pos = Point( x = current_pose['position'][0],
        #                y = current_pose['position'][1],
        #                z = current_pose['position'][2] )
        pos = position
        pose = PoseStamped(header=hdr, pose=Pose(\
                position = pos, orientation = current_pose['orientation'] ) )
        self.ikreq.pose_stamp.append(pose)
        try:
            rospy.wait_for_service(self.ik_name, 5.0)
            resp = self.iksvc(self.ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

        if (resp.isValid[0]):
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            # print limb_joints
            self.limb.move_to_joint_positions(limb_joints)
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")

    def to_joint_positions(self, positions):
        self.limb.move_to_joint_positions(positions)

def main():
    print("Initializing node... ")
    rospy.init_node("move_arm")

    move = MoveArm()

    print "move to (0.5, 0.0, 0.3)"
    move.to_XYZ(Point(0.5, 0.0, 0.3))

    rospy.sleep(1.0)
    print "move to (0.5, 0.0, 0.4)"
    move.to_XYZ(Point(0.5, 0.0, 0.4))

    rospy.sleep(1.0)
    print "move to (0.5, 0.0, 0.2)"
    move.to_XYZ(Point(0.5, 0.0, 0.2))


if __name__ == '__main__':
    main()
