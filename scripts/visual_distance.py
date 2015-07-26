#!/usr/bin/env python
# 2015 Juxi Leitner <j.leitner@roboticvision.org>
# Australian Centre for Robotic Vision (ACRV)

import rospy
import baxter_interface
import baxter_external_devices
import cv2
import cv_bridge

from baxter_interface import CHECK_VERSION
from std_msgs.msg import String, Float
from sensor_msgs.msg import Image

class VisualDistance(object):
    """
    """

    def __init__(self, device=0):
        self.is_stopping = False
        self.bridge = cv_bridge.CvBridge()
        self.cam = cv2.VideoCapture(device)
        self.rate = rospy.Rate(10) # Hz

        rospy.loginfo('Starting the visual distance node...')
        self.pub = rospy.Publisher('/vis_dist', Float, latch=True, queue_size=1)
        rospy.on_shutdown(self.clean_shutdown)

        # start the controller
        self.update()

    def update(self):
        """ calculate the camera image/visual distance """
        # perform the calculation of the distance between the end-effector
        # and the target/goal visible in the image
        while not self.is_stopping and self.cam.isOpened():
            # retrive camera image
            ret, frame = self.cam.read()
            if ret == False: break

            # TODO do some operation on the camera image here!
            # detect the end effector in pixel coords
            # detect the target position in pixel coords

            msg = 1.0
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

    node_name = "vis_dist"
    rospy.init_node(node_name)

    # start the node
    arm = VisualDistance()

    # when it returns it is done
    print("Done.")


##########################
if __name__ == '__main__':
    main()
