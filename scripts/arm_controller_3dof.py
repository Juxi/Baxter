#!/usr/bin/env python
# 2015 Juxi Leitner <j.leitner@roboticvision.org>
# Australian Centre for Robotic Vision (ACRV)

import argparse
import rospy
import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION
from std_msgs.msg import String

class ArmController3DOF(object):
    """ 3DOF Arm controller

        A controller class for the limbs of Baxter, for simple control studies.
        The motion is restricted to 3 degrees-of-freedoms and limites the robot
        to a (vertical) plane.
    """

    def __init__(self, limb, moveToInitial=True):
        self.is_stopping = False
        self.rate = rospy.Rate(10) # 4 Hz
        self.limb = limb
        self.joint_names = { 'left_s1', 'left_e1', 'left_w1' }
        self.delta_angle = 0.2  # Baxter joint angles are in radians

        rospy.loginfo('Starting the 3DOF controller for the %s limb', limb)

        # TODO create a publisher for the finished action announcer
        #   ...

        # TODO look at different msg types (e.g. JointState)
        # Subscriber to a topic where the 3DOF commmands are sent
        # topic_name = "/3dof_baxter_arm_controller/" + self.limb + "/in"
        # rospy.Subscriber(topic_name, String, self.pose_received)

        # for deepmind tryout
        topic_name = "/action"
        rospy.Subscriber(topic_name, String, self.action_received)

        rospy.on_shutdown(self.clean_shutdown)

        # active Baxter
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        self.interface = baxter_interface.Limb(limb)

        # magic constant: joint speed :)
        self.interface.set_joint_position_speed(0.35)

        if moveToInitial: self.move_to_initial()

        # initialize the desired position with the current position
        angles = self.interface.joint_angles()
        self.desired_joint_pos = { key: angles[key] for key in self.joint_names }
        rospy.loginfo('Initialization finished!')

        # start the controller
        # self.control()

        rospy.spin()    # only reactive control, wait for action cmd received


    def status(self):
        angles = self.interface.joint_angles()
#        robot_pose = self.interface.joint_angles()
        robot_pose = { key: angles[key] for key in self.joint_names }
        return robot_pose


    def move_to_initial(self):
        rospy.loginfo('Moving the %s limb to initial position ...', self.limb)
        limb_joint_pos = { 'left_s0': -0.5, 'left_s1': -1.0,
                           'left_w0':  0.0, 'left_w1':  1.5, 'left_w2': 0.0,
                           'left_e0':  0.0, 'left_e1':  0.5 }
        self.interface.move_to_joint_positions(limb_joint_pos)

        return limb_joint_pos
        # TODO look at this and see what the deep code needs
#        limb_joint_pos = zip(self.joint_names, { 0, 0, 0 } )
#        self.interface.move_to_joint_positions(limb_joint_pos))


    # def control(self):
    #     # perform the control of the arm based on the current 'policy'
    #     while not self.is_stopping:
    #         self.update()
    #         # sleep to create a control frequency
    #         self.rate.sleep()


    def update(self):
        """ updating the controller
            using a simple joint position controller """

        # get current joint state
        error = dict()
        limb_joint_pos = dict()

        for joint in self.joint_names:
            limb_joint_pos[joint] = self.desired_joint_pos[joint]

        # print "new joint position: ", limb_joint_pos
        self.interface.move_to_joint_positions(limb_joint_pos)


    def set_desired(self, joint_name, joint_delta):
        jnt_pos = self.desired_joint_pos
        jnt_pos[joint_name] = jnt_pos[joint_name] + joint_delta
        return jnt_pos

    def action_received(self, data):
        rospy.loginfo("received: %s", data.data)
        bindings = {
        #   key: (function, args, description)
            'btn1':  (self.set_desired, ['left_s1', +self.delta_angle]),
            'btn2':  (self.set_desired, ['left_s1', -self.delta_angle]),
            'btn3':  (self.set_desired, ['left_e1', +self.delta_angle]),
            'btn4':  (self.set_desired, ['left_e1', -self.delta_angle]),
            'btn5':  (self.set_desired, ['left_w1', +self.delta_angle]),
            'btn6':  (self.set_desired, ['left_w1', -self.delta_angle]),
            'reset': (self.move_to_initial, []),
        }

        action_rcvd = data.data # maybe need some more processing here?
        if action_rcvd in bindings:
            cmd = bindings[action_rcvd]
            self.desired_joint_pos = cmd[0](*cmd[1])
            self.update()
        else:
            rospy.loginfo("Unknown command received! (no binding found)")


#     def pose_received(self, data):
#         rospy.loginfo("received: %s", data.data)
#         # TODO do something real here
#         # debug
# #        self.desired_joint_pos = zip(self.joint_names, { 0, 0, 0 } )
#         self.desired_joint_pos = { key: 0.0 for key in self.joint_names }


    def clean_shutdown(self):
        # stop the controller and the ROS node
        self.is_stopping = True
        print("\nExiting 3DOFArmController!")


################################################################################
def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument('-l','--limb', dest='limb', default='left',
                        choices=['right','left'],
                        help='Limb selection (default: left)' )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node ... ")

    node_name = args.limb + "_arm_controller_3dof"
    rospy.init_node(node_name)

    # start the controller
    arm = ArmController3DOF(args.limb)

    # when it returns it is done
    print("Done.")


##########################
if __name__ == '__main__':
    main()
