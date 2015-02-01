#!/usr/bin/python
# by Juxi Leitner
#----------------------------------------------------------------------------------

import os
import sys

import rospy

import cv2
import cv_bridge

import baxter_interface

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import Image

import time
import numpy as np

global my_camera

#class baxter_camera_subscriber:
def open_camera(camera_name):
	my_camera = baxter_interface.CameraController(camera_name) #"right_hand_camera")
	my_camera.resolution = (1280, 800)
	my_camera.open()

def read_image_callback(data):
	img = cv_bridge.CvBridge().imgmsg_to_cv2(data, "bgr8")		
	display_image(img)

def display_image(img):
	msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
	pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
	pub.publish(msg)

def send_image(path):
	img = cv2.imread(path)
	msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="passthrough")
	pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
	pub.publish(msg)
	print "publishing image"
	rospy.sleep(1)		# allow for update of the screen

def main():
	rospy.init_node('show_camera')
    
	args = rospy.myargv()

	if( args[1] != "--limb" or (args[2] != "right" and args[2] != "left" )):
		print "Usage: show_camera.py --limb [right|left]"
		sys.exit()

	camera_name = args[2] + "_hand_camera"
	open_camera(camera_name)
	# set callback to
	rospy.Subscriber('/cameras/' + camera_name + '/image', Image, read_image_callback, queue_size=1)

	try: #while not rospy.is_shutdown():
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down 'show_camera'..."
	#img = read_image() #my_camera
	#send_image("frame0000.jpg")
	# display_image(img)
#	rospy.sleep(1)



if __name__ == '__main__':
    sys.exit(main())
