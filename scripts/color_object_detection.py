#!/usr/bin/python
# Author: Juxi Leitner <j.leitner@qut.edu.au>, (c) 2015
# based on code by Allen Liang <h2.liang@student.qut.edu.au>
# CopyPolicy: GPL v2

import argparse
import struct
import sys

import baxter_interface

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import math
import time
import numpy as np

import rospy
import ast
from std_msgs.msg import (
  String,
  Header,
)

import time
import threading

#	def mainProgram(self):

#		# Set Baxter Robot Motions (Scan Path)
#		# Create an instance of scanPath
#		scanMotion = scanPath()
#
		# Raise the left hand to avoid crash with right hand
#		scanMotion.getReady()
#		
#		test = scanPublisher()
#
#
#		# Start doing scanning
#		scanMotion.scan()
#		test.programFinished = True
#		objectLocation = test.getObjectPosition()
#
#		for i in range(len(objectLocation)):
#			scanMotion.setPosition(objectLocation[i][0],objectLocation[i][1],0.080000000000)
#			scanMotion.movePosition()
#			time.sleep(4)
		# Main scanning function

		# Reset position
#		scanMotion.setDefault()
#
#		print "Program finished"



import argparse
import struct
import sys

from matplotlib import pyplot as plt
import numpy as np
import baxter_interface
from copy import deepcopy
from os import system
import rospy
import ast

import cv2
from cv_bridge import CvBridge, CvBridgeError
from baxter_interface.camera import CameraController

import rospkg
import math
import time

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from sensor_msgs.msg import (
  Image,
)

from std_msgs.msg import (
  String,
  Float64MultiArray,
  Header,
)

class colorObjectDetection:
	def __init__(self):
	        self.is_stopping = False
		camera_topic = '/cameras/right_hand_camera/image'
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber(camera_topic,Image, self.imageReceived)
		self.pub = rospy.Publisher('object_position', String, queue_size=2)
#        self._debug = debug
	        rospy.loginfo('Starting the object detector..')

	        rospy.on_shutdown(self.clean_shutdown)

		self.object_detected = False
		self.object_location = [0,0]
		self.is_stopping = False

		rospy.spin()


	def clean_shutdown(self):
	# stop it !
		self.is_stopping = True
		print("\nExiting Detector...")
        #self.interface.stop()



	def positionCalculation(self,x,y,pX,pY):
		# Calculate the distance on the x and y axis in the image
		xDistance = pX - self.width/2.0
		yDistance = pY - self.height/2.0

		# Convert the distance for arm based on experiment
		xMovement = math.fabs(yDistance / 1600.0)
		yMovement = math.fabs(xDistance / 1600.0)

		# Move based on the position of the Y
		if (pY < self.height/2.0):
			x = x + xMovement
		elif (pY > self.height/2.0):
			x = x - xMovement

		# Move based on the position of the X
		if (pX < self.width/2.0):
			y = y + yMovement
		elif (pX > self.width/2.0):
			y = y - yMovement

		self.object_location = [ x , y ]
		#self.objectAddress.append([x,y])




	def objectDetect(self,imageData):
		right_hand = baxter_interface.Limb('right')

		# Get the size of the frame (height: 720 width: 1280)
		self.height, self.width = imageData.shape[:2]

		# Set central box position
		self.centralBoxStartX = self.width/2 - 85
		self.centralBoxEndX = self.centralBoxStartX + 170
		self.centralBoxStartY = self.height/2 - 85
		self.centralBoxEndY = self.centralBoxStartY + 170

		# Draw central box
		# cv2.rectangle(imageData, (self.centralBoxStartX,self.centralBoxStartY),(self.centralBoxEndX,self.centralBoxEndY),(255,255,255),3)

		# Convert the current frame to HSV
		hsv = cv2.cvtColor(imageData, cv2.COLOR_BGR2HSV)

		lower_red = np.array([170, 35, 35])
		upper_red = np.array([200, 255, 255])

		# Create a binary image, where anything red appears white and everything else is black
		mask = cv2.inRange(hsv, lower_red, upper_red)

		# Get rid of background noise using erosion and fill in the
		# holes using dilation and erode the final image on last time
		element = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
		mask = cv2.erode(mask,element, iterations=2)
		mask = cv2.dilate(mask,element,iterations=2)
		mask = cv2.erode(mask,element)
	
		# Create Contours for all red objects
		contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		maximumArea = -1
		bestContour = None
		for contour in contours:
			currentArea = cv2.contourArea(contour)
			if currentArea > maximumArea:
			    bestContour = contour
			    maximumArea = currentArea

		# Create a bounding box around the biggest red object
		if bestContour is not None:
			x,y,w,h = cv2.boundingRect(bestContour)
			cv2.rectangle(imageData, (x,y),(x+w,y+h), (0,0,255), 3)

			# Find the central point position of object founded
			px_u = x + 0.5 * w # Start X + half width
			px_v = y + 0.5 * h # Start Y + half height
			self.object_found = True
			self.object_location = [ px_u, px_v ]
		else:
			self.object_location = [0,0]
			self.object_found = False


	def imageReceived(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError, e:
			print e

		self.objectDetect(cv_image)
		#cv2.imshow("Baxter Right Hand Image", cv_image)
		#cv2.waitKey(3)
		self.publishLocation()
		


	def publishLocation(self):
#		rate = rospy.Rate(1) #10hz
#		while not rospy.is_shutdown():
		hello_str = str(self.object_location)
		rospy.loginfo(hello_str)
		self.pub.publish(hello_str)
		#rate.sleep()




	def getObjectPosition(self):
		return self.latest_position
		

if __name__ == '__main__':
	print("Initializing node... ")
	rospy.init_node('ColorObjectDetector')

	# start the detector
	det = colorObjectDetection()\
	# when it returns it's done
	print("Done.")


