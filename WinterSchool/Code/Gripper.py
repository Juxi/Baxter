#!/usr/bin/python
##########################################################################
# Simple ROS listener in Python
# copyleft: Juxi Leitner <j.leitner@roboticvision.org>
# Australian Centre for Robotic Vision (ACRV)
# Queensland University of Techonology (QUT)
#
# Coded for the QUT Robotics and Autonomous Systems Winter
# School, 2015, Brisbane
# http://Juxi.net/WinterSchool/2015/
#
# modified from the RethinkRobotics SDK tutorials
#
##########################################################################

import argparse
import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

class Gripper(object):
    def __init__(self):
        self.initialize_gripper()

    def clean_shutdown(self):
        if not self.init_state:
            print("Disabling robot...")
            self.rs.disable()
        print("Exiting example.")

    def initialize_gripper(self):
        # initialize interfaces
        print("Getting robot state... ")
        self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self.init_state = self.rs.state().enabled
        self.gripper = baxter_interface.Gripper('left', CHECK_VERSION)
#        self.right = baxter_interface.Gripper('right', CHECK_VERSION)
        rospy.on_shutdown(self.clean_shutdown)

    def capability_warning(self, cmd):
        msg = ("%s %s - not capable of '%s' command" %
               (self.gripper.name, self.gripper.type(), cmd))
        rospy.logwarn(msg)

    def offset_position(self, offset):
        if self.gripper.type() != 'electric':
            capability_warning(self.gripper, 'command_position')
            return
        current = self.gripper.position()
        self.gripper.command_position(current + offset)

    def offset_holding(self, offset):
        if self.gripper.type() != 'electric':
            capability_warning(self.gripper, 'set_holding_force')
            return
        current = self.gripper.parameters()['holding_force']
        self.gripper.set_holding_force(current + offset)

    def offset_moving(self, offset):
        if self.gripper.type() != 'electric':
            capability_warning(self.gripper, 'set_moving_force')
            return
        current = self.gripper.parameters()['moving_force']
        self.gripper.set_moving_force(current + offset)

    def offset_velocity(self, offset):
        if self.gripper.type() != 'electric':
            capability_warning(self.gripper, 'set_velocity')
            return
        current = self.gripper.parameters()['velocity']
        self.gripper.set_velocity(current + offset)

    def offset_dead_band(self, offset):
        if self.gripper.type() != 'electric':
            capability_warning(self.gripper, 'set_dead_band')
            return
        current = self.gripper.parameters()['dead_zone']
        self.gripper.set_dead_band(current + offset)


    def map_keyboard(self):

        bindings = {
        #   key: (function, args, description)
            'r': (self.gripper.reboot, [], "left: reboot"),
            'c': (self.gripper.calibrate, [], "left: calibrate"),
            'q': (self.gripper.close, [], "left: close"),
            'w': (self.gripper.open, [], "left: open"),
            '[': (self.gripper.set_velocity, [100.0], "left:  set 100% velocity"),
            ']': (self.gripper.set_velocity, [30.0], "left:  set 30% velocity"),
            's': (self.gripper.stop, [], "left: stop"),
        }

        done = False
        print("Enabling robot... ")
        self.rs.enable()
        print("Controlling grippers. Press ? for help, Esc to quit.")
        while not done and not rospy.is_shutdown():
            c = baxter_external_devices.getch()
            if c:
                if c in ['\x1b', '\x03']:
                    done = True
                elif c in bindings:
                    cmd = bindings[c]
                    cmd[0](*cmd[1])
                    print("command: %s" % (cmd[2],))
                else:
                    print("key bindings: ")
                    print("  Esc: Quit")
                    print("  ?: Help")
                    for key, val in sorted(bindings.items(),
                                               key=lambda x: x[1][2]):
                            print("  %s: %s" % (key, val[2]))

        # force shutdown call if caught by key handler
        rospy.signal_shutdown("Example finished.")


def main_keyboard():
    """RSDK Gripper Example: Keyboard Control

    Use your dev machine's keyboard to control and configure
    Baxter's grippers.

    Run this example to command various gripper movements while
    adjusting gripper parameters, including calibration, velocity,
    and force. Uses the baxter_interface.Gripper class and the
    helper function, baxter_external_devices.getch.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main_keyboard.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_gripper_keyboard")

    grip = Gripper()
#    grip.initialize_gripper()
    grip.map_keyboard()


if __name__ == '__main__':
    main_keyboard()
