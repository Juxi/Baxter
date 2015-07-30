#!/usr/bin/env python
# 2015 Juxi Leitner <j.leitner@roboticvision.org>
# Australian Centre for Robotic Vision (ACRV)

import argparse
import rospy
import baxter_interface
import baxter_external_devices
import os
import os.path


import numpy as np
import cv2
import cv_bridge


from baxter_interface import CHECK_VERSION
from std_msgs.msg import String

from sensor_msgs.msg import Image


class ArmController3DOF(object):
    """ 3DOF Arm controller

        A controller class for the limbs of Baxter, for simple control studies.
        The motion is restricted to 3 degrees-of-freedoms and limites the robot
        to a (vertical) plane.
    """

    def __init__(self, limb, moveToInitial=False, device=0):
        self.is_stopping = False
        self.rate = rospy.Rate(100) # 10 Hz
        self.limb = limb
        self.joint_names = { 'left_s1', 'left_e1', 'left_w1' }
        self.delta_angle = 0.02  # Baxter joint angles are in radians

        rospy.loginfo('Starting the 3DOF controller for the %s limb', limb)

        #self.crop_width, self.crop_height = 160, 320
        self.crop_width, self.crop_height = 320, 480

        self.bridge = cv_bridge.CvBridge()
        self.cam = cv2.VideoCapture(device)

        rospy.loginfo('Starting the usbcam_filewriter...')

        # TODO create a publisher for the finished action announcer
        #   ...

        # TODO look at different msg types (e.g. JointState)
        # Subscriber to a topic where the 3DOF commmands are sent
        # topic_name = "/3dof_baxter_arm_controller/" + self.limb + "/in"
        # rospy.Subscriber(topic_name, String, self.pose_received)

        # for deepmind tryout
        file_name = "action.txt"
        #rospy.Subscriber(topic_name, String, self.action_received)

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
        self.control()

        rospy.spin()    # only reactive control, wait for action cmd received


    def status(self):
        angles = self.interface.joint_angles()
#        robot_pose = self.interface.joint_angles()
        robot_pose = { key: angles[key] for key in self.joint_names }
        return robot_pose


    def move_to_initial(self):
        rospy.loginfo('Moving the %s limb to initial position ...', self.limb)
        limb_joint_pos = { 'left_s0': -0.5, 'left_s1':  0.0,
                           'left_w0':  0.0, 'left_w1':  0.0, 'left_w2': 0.0,
                           'left_e0':  0.0, 'left_e1':  0.0 }
        self.interface.move_to_joint_positions(limb_joint_pos)

        return limb_joint_pos
        # TODO look at this and see what the deep code needs
#        limb_joint_pos = zip(self.joint_names, { 0, 0, 0 } )
#        self.interface.move_to_joint_positions(limb_joint_pos))


    def get_image(self):
        if self.cam.isOpened():
            # retrive camera image
            ret, frame = self.cam.read()
            if ret == False: 
                rospy.loginfo("no camera found")
                return

            # TODO do some operation on the camera image here!

            # Crop from x, y, w, h -> 100, 200, 300, 400
            # img[200:400, 100:300]

            #height, width, channels = img.shape
            #middle_h, middle_w = height/2, width/2
            height, width = frame.shape
            
            #x = frame_height/2 - self.crop_height/2
            #y = frame_width/3 - self.crop_width/2

            # horizontal
            crap_w, crap_h = (320, 480)
#            start_pt = (width/3-crap_w/3,height/2-crap_h/2)
#            end_pt = (width/3+2*crap_w/3,height/2-crap_h/2)
#            cv2.line(frame, start_pt, end_pt, colour, 1)

#            start_pt = (width/3-crap_w/3,height/2+crap_h/2)
#            end_pt = (width/3+2*crap_w/3,height/2+crap_h/2)
#            cv2.line(frame, start_pt, end_pt, colour, 1)


            # vertical
#            start_pt = (width/3-crap_w/3,height/2+crap_h/2)
#            end_pt = (width/3-crap_w/3,  height/2-crap_h/2)
#            cv2.line(frame, start_pt, end_pt, colour, 1)

 #           start_pt = (width/3+2*crap_w/3,height/2+crap_h/2)
 #           end_pt = (width/3+2*crap_w/3,  height/2-crap_h/2)
 #           cv2.line(frame, start_pt, end_pt, colour, 1)


#            cropped_frame = frame[height/2-crap_h/2:height/2+crap_h/2, width/3-crap_w/3:width/3+2*crap_w]
            init_h = 0
            init_w = width/3-crap_w/3
            cropped_frame = frame[init_h:init_h+crap_h, init_w:init_w+crap_w]

            # resizing it 160x240
            #small = cv2.resize(image, (0,0), fx=0.5, fy=0.5) 
            #and this will resize the image to have 100 cols (width) and 50 rows (height):
            cropped_frame = cv2.resize(cropped_frame, (crap_w/2,crap_h/2)) 

#           img[273:333, 100:160] = ball\
            frame = cv2.imread('../../sample_view-simulator.png', 0)
#            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # frame[40:280, 0:160] = cropped_frame      <- standard seems to work

#            frame[40:280, 40:160] = cropped_frame[0:240, 40:160]       <- cut off left
            frame[65:280, 40:160] = cropped_frame[25:240, 40:160]    
            cropped_frame = frame


            #cropped_frame = frame[middle_h-crop_width/2:middle_w-crop_height/2,

            # convert frame and publish message
            # msg = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            # msg = self.bridge.cv2_to_imgmsg(cropped_frame, "mono8")
            #self.pub.publish(msg)
            cv2.imwrite('cam.png', cropped_frame)

    def write_reward(self):
        file = open("reward.txt", "w")
        file.write("-1")
        file.close()



    def control(self):
        # perform the control of the arm based on the current 'policy'
        self.get_image()
        self.write_reward()

        file_name = "action.txt"
        while not self.is_stopping:
            # check if action file exists
            if os.path.isfile(file_name):
                file = open(file_name, 'r')
                self.action_received(file.readline())
                file.close()

                os.remove(file_name)

                self.get_image()
                self.write_reward()

            else:
                # sleep to create a control frequency
                self.rate.sleep()
                self.cam.read()

        self.cam.release()


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
        rospy.loginfo("received: %s", data)
        bindings = {
        #   key: (function, args, description)
            #'0': (self.set_desired, ['left_s1', 0]),
#            'btn1':  (self.set_desired, ['left_s1', +self.delta_angle]),
#            'btn2':  (self.set_desired, ['left_s1', -self.delta_angle]),
#            'btn3':  (self.set_desired, ['left_e1', +self.delta_angle]),
#            'btn4':  (self.set_desired, ['left_e1', -self.delta_angle]),
#            'btn5':  (self.set_desired, ['left_w1', +self.delta_angle]),
#            'btn6':  (self.set_desired, ['left_w1', -self.delta_angle]),
            'btn1':  (self.set_desired, ['left_s1', -self.delta_angle]),
            'btn2':  (self.set_desired, ['left_s1',  self.delta_angle]),
            'btn3':  (self.set_desired, ['left_e1', -self.delta_angle]),
            'btn4':  (self.set_desired, ['left_e1',  self.delta_angle]),
            'btn5':  (self.set_desired, ['left_w1', -self.delta_angle]),
            'btn6':  (self.set_desired, ['left_w1',  self.delta_angle]),

            'reset': (self.move_to_initial, []),
        }

        action_rcvd = data # maybe need some more processing here?
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
    arm = ArmController3DOF(args.limb, moveToInitial=True)

    # when it returns it is done
    print("Done.")


##########################
if __name__ == '__main__':
    main()
