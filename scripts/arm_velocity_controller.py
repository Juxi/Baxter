#!/usr/bin/env python
# 2015 Juxi Leitner <j.leitner@roboticvision.org>
# building on the Rethink Robotics Baxter Example:
# RSDK Joint Position Example: Keyboard Control

import argparse
import rospy

import baxter_interface
import baxter_external_devices

from std_msgs.msg import String


MAX_VAL = 0.75

testPose = {'left_s0':  0.4544,
            'left_s1': -0.9376,
            'left_e0': -1.0070,
            'left_e1':  1.6574,
            'left_w0':  0.6293,
            'left_w1':  1.1301,
            'left_w2':  0.4191,
            'right_s0': 0.4544,
            'right_s1':-0.9375,
            'right_e0':-1.0071,
            'right_e1': 1.6574,
            'right_w0': 0.6293,
            'right_w1': 1.1301,
            'right_w2': 0.4191
}

class ArmController(object):
    """
    """

    def __init__(self, limb, moveToNeutralAtStart=False, debug=False): #, master=True): #dunno if master is needed yet
        self.is_stopping = False
        self.rate = rospy.Rate(10) # 10 Hz
        self.l = .5
        self._debug = debug
        self.limb = limb
        rospy.loginfo('Starting the controller for the %s limb', limb)

        # TODO create a publisher for the controller state
        #   ...
        # Subscriber to a topic provding the control policy (e.g. final position)
        rospy.Subscriber("baxter_arm_controller/" + self.limb + "/in", String, self.control_received)
        rospy.on_shutdown(self.clean_shutdown)

        self.interface = baxter_interface.Limb(limb)
        # magic constant :)
        self.interface.set_joint_position_speed(0.2)
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
        global MAX_VAL
        # get current state of the robot

        # simple joint velocity controller
        # get current joint state
        error = dict()
        limb_joint_pos = limb_joint_vel = dict()
        for joint in self.interface.joint_names():
            limb_joint_pos[joint] = self.interface.joint_angle(joint)
            if self._debug: 
                print joint, ":" , self.interface.joint_angle(joint) 

        # calculate error to desired joint states
        for joint in self.interface.joint_names():
            error[joint] = (self.desired_joint_pos[joint] - limb_joint_pos[joint])
            if self._debug:
                print "error: ", joint, round(error[joint], 2)

        # linear update
        for joint in self.interface.joint_names():
            limb_joint_vel[joint] = self.l * error[joint]
            # TODO make sure it's reasonable values
            limb_joint_vel[joint] = max( -MAX_VAL, min( MAX_VAL, limb_joint_vel[joint]))

        if self._debug: 
            print "new joint velocities: ", limb_joint_vel
            print "-"*10

        if min(error.values()) < -0.15 or max(error.values()) > 0.15:
            self.interface.set_joint_velocities(limb_joint_vel)
        else:
            #print "error is small enough, no control output"
            print ".",


    def control_received(self, data):
        rospy.loginfo(rospy.get_called_id() + "received: %s", data.data)
        # update self.desired_joint_pos[joint]
        self.set_desired_joint_pos(data)

    def set_desired_joint_pos(self, data):
        self.desired_joint_pos = dict()
        for joint in ['_s0', '_s1', '_e0', '_e1', '_w0', '_w1', '_w2']:
            self.desired_joint_pos[self.limb + joint] = data[self.limb + joint]

    def clean_shutdown(self):
        # stop controller !
        self.is_stopping = True
        print("\nExiting ArmController...")
        self.interface.exit_control_mode()



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
    ctrl = ArmController(args.limb)#, debug=True)

    # when it returns it's done
    print("Done.")


if __name__ == '__main__':
    main()
