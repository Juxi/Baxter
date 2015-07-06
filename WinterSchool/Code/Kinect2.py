#!/usr/bin/python
##########################################################################
# Simple listener to PointCloud2 (from ROS)
# copyleft: Juxi Leitner <j.leitner@roboticvision.org>
# Australian Centre for Robotic Vision (ACRV)
# Queensland University of Techonology (QUT)
#
# Coded for the QUT Robotics and Autonomous Systems Winter
# School, 2015, Brisbane
# http://Juxi.net/WinterSchool/2015/
#
##########################################################################

#!/usr/bin/env python
import rospy
import pcl
import sensor_msgs.point_cloud2 as pc2
import numpy


def depthimage_received_callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s bytes", len(data.data))

    cloud = pcl.PointCloud()
    big_array = [] #  empty regular list
    for (x,y,z) in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
        big_array.append([x, y, z])
    # creat numpy array
    np_array = numpy.array(big_array, dtype=numpy.float32)    
    cloud.from_array(np_array)


    # ADD SOME CODE TO WORK ON THE POINT CLOUD
    cloud.to_file("cloud.pcd")


    rospy.signal_shutdown("single read")

    

def kinect_listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('kinect_listener', anonymous=True)

    port = "/kinect2_head/depth_lowres/points" #/kinect2_head/depth_lowres/image"
    rospy.Subscriber(port, pc2.PointCloud2, depthimage_received_callback)
    rospy.loginfo("Listening to " + port)

    # spin() simply keeps python from exiting until this node is
    # stopped, with e.g. CTRL-C
    rospy.spin()



if __name__ == '__main__':
    kinect_listener()

