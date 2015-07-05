#!/usr/bin/python
################################################################
# Simple example of OpenCV usage with Python
# copyleft: Juxi Leitner <j.leitner@roboticvision.org>
# Australian Centre for Robotic Vision (ACRV)
# Queensland University of Techonology (QUT)
#
# Coded for the QUT Robotics and Autonomous Systems Winter
# School, 2015, Brisbane
# http://Juxi.net/WinterSchool/2015/
#
# based on OpenCV tutorials and code by Ben Upcroft
#
################################################################

import cv2
import numpy as np
from matplotlib import pyplot as plt

def thresholding_test(img):
    ret,th1 = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
    th2 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C, \
            cv2.THRESH_BINARY,11,2)
    th3 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
            cv2.THRESH_BINARY,11,2)
    titles = ['Original Image', 'Global Thresholding (v = 127)',
            'Adaptive Mean Thresholding', 'Adaptive Gaussian Thresholding']
    images = [img, th1, th2, th3]
 
    for i in xrange(4):
        plt.subplot(2,2,i+1),plt.imshow(images[i],'gray')
        plt.title(titles[i])
        plt.xticks([]),plt.yticks([])
    plt.show()

img = cv2.imread('../TestData/testframe.jpg',0)
img = cv2.medianBlur(img,5)
 
thresholding_test(img)

