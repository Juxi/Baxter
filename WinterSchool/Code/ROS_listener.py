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
# slightly modified from the ROS tutorials
# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(python)
#
##########################################################################

#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image


def image_received_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s bytes", len(data.data))

    # converting the message to OpenCV data format
    cv_img = cv_bridge.CvBridge().imgmsg_to_cv2(data, "bgr8")

    # ADD SOME CODE TO WORK ON THE IMAGES RECEIVED

    

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    port = "/cameras/left_hand_camera/image"
    rospy.Subscriber(port, Image, image_received_callback)
    rospy.loginfo("Listening to " + port)

    # spin() simply keeps python from exiting until this node is
    # stopped, with e.g. CTRL-C
    rospy.spin()



if __name__ == '__main__':
    listener()

