#!/usr/bin/python
##########################################################################
# Simple example of OpenCV usage with Python
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
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
