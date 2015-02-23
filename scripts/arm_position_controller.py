#!/usr/bin/env python
# 2015 Juxi Leitner
# building on the Rethink Robotics Baxter Example:
# RSDK Joint Position Example: Keyboard Control

import argparse
import rospy

import baxter_interface
import baxter_external_devices

from std_msgs.msg import String


testPose = {'left_s0': 0.454441807892,
            'left_s1': -0.937645755524,
            'left_e0': -1.0070583861,
            'left_e1': 1.65746623942,
            'left_w0': 0.629315617511,
            'left_w1': 1.13016034418,
            'left_w2': 0.419160249811
}

class ArmController(object):
    """
    """

    def __init__(self, limb, moveToNeutralAtStart=False): #, master=True): #dunno if master is needed yet
        self.is_stopping = False
        self.rate = rospy.Rate(10) # 10 Hz
        self.l = .1
        self.limb = limb
        rospy.loginfo('Starting the controller for the %s limb', limb)

        # TODO create a publisher for the controller state
        #   ...

        # Subscriber to a topic provding the control policy (e.g. final position)
        # TODO look at different msg types (e.g. JointState)
        rospy.Subscriber("baxter_arm_controller/" + self.limb + "/in", String, self.control_received)
        rospy.on_shutdown(self.clean_shutdown)

        self.interface = baxter_interface.Limb(limb)
        
        # magic constant :)
        self.interface.set_joint_position_speed(0.35)
        
        # TODO fixed target for now
        self.desired_joint_pos = testPose # dict()

	    # TODO think about this?
        if moveToNeutralAtStart: 
            rospy.loginfo('Moving the %s limb to a neutral position ...', limb)
            self.interface.move_to_neutral()

        rospy.loginfo('Initialization finished!')
        # start the controller
        self.control()
        

    def control(self):
        # perform the control of the arm based on the current 'policy'
        while not self.is_stopping:
            # TODO: do some error checking?!
            self.update()
            # sleep to create a control frequency
            self.rate.sleep()

    def update(self): #updating the controlller
        # get current state of the robot

        # simple joint velocity controller
        # get current joint state
        error = dict()
        limb_joint_pos = dict()
        for joint in self.interface.joint_names():
            limb_joint_pos[joint] = self.interface.joint_angle(joint)
            print joint, ":" , self.interface.joint_angle(joint) 

        # calculate error to desired joint states
        for joint in self.interface.joint_names():
            error[joint] = (self.desired_joint_pos[joint] - limb_joint_pos[joint])
            print "error: ", joint, round(error[joint], 2)

        # linear update
        for joint in self.interface.joint_names():
            limb_joint_pos[joint] = limb_joint_pos[joint] + self.l * error[joint]

        print "new joint position: ", limb_joint_pos
        self.interface.move_to_joint_positions(limb_joint_pos)


    def control_received(self, data):
        rospy.loginfo(rospy.get_called_id() + "received: %s", data.data)
        # update self.desired_joint_pos[joint]

    def clean_shutdown(self):
        # stop controller !
        self.is_stopping = True
        print("\nExiting ArmController...")




###########################################
def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument('-l','--limb', dest='limb', default='right',
                        choices=['right','left'],
                        help='Limb selection (default: right)' )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    node_name = args.limb + "arm_controller"
    rospy.init_node(node_name)

    # start the controller
    ArmController(args.limb)

    # when it returns it's done
    print("Done.")


if __name__ == '__main__':
    main()
