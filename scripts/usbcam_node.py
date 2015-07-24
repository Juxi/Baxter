#!/usr/bin/env python
# 2015 Juxi Leitner <j.leitner@roboticvision.org>
# Australian Centre for Robotic Vision (ACRV)

import argparse
import rospy
import baxter_interface
import baxter_external_devices
import cv2
import cv_bridge

from baxter_interface import CHECK_VERSION
from std_msgs.msg import String
from sensor_msgs.msg import Image

class CameraNode(object):
    """
    """

    def __init__(self, device=0):
        self.is_stopping = False
        self.rate = rospy.Rate(20) # Hz

        self.bridge = cv_bridge.CvBridge()
        self.cam = cv2.VideoCapture(device)

        rospy.loginfo('Starting the cam_node...')
        self.pub = rospy.Publisher('/cam_node', Image, latch=True, queue_size=1)
        rospy.on_shutdown(self.clean_shutdown)

        # start the controller
        self.update()

    def update(self):
        """ updating the camera image """
        # perform the control of the arm based on the current 'policy'
        while not self.is_stopping and self.cam.isOpened():
            # retrive camera image
            ret, frame = self.cam.read()
            if ret == False: break

            # TODO do some operation on the camera image here!

            # convert frame and publish message
            # msg = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            msg = self.bridge.cv2_to_imgmsg(frame, "mono8")
            self.pub.publish(msg)

            # sleep to create a control frequency
            self.rate.sleep()
        self.cam.release()


    def clean_shutdown(self):
        # stop the controller and the ROS node
        print("\nExiting Cam Node!")
        self.is_stopping = True


################################################################################
def main():
    print("Initializing node ... ")

    node_name = "usbcam_node"
    rospy.init_node(node_name)

    # start the controller
    arm = CameraNode()

    # when it returns it is done
    print("Done.")


##########################
if __name__ == '__main__':
    main()
