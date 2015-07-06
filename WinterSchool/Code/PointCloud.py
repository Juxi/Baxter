#!/usr/bin/python
##########################################################################
# Simple PointCloud2 parser
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
import pcl

p = pcl.PointCloud()
p.from_file("../TestData/kinect2_tower.pcd")

print(p.size)
seg = p.make_segmenter()

fil = p.make_passthrough_filter()
fil.set_filter_field_name("z")
fil.set_filter_limits(0, 1.5)
p = fil.filter()

seg = p.make_segmenter_normals(ksearch=50)
seg.set_optimize_coefficients(True)
seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
seg.set_normal_distance_weight(0.1)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_max_iterations(100)
seg.set_distance_threshold(0.03)
indices, model = seg.segment()

print(model)

cloud = p.extract(indices, negative=False)
cloud.to_file("newcloud.pcd")
