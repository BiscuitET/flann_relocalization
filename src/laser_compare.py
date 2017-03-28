#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
import os
import message_filters
import time
from numpy import *

from sys import *
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from amcl.msg import PoseWithWeightArray
from amcl.msg import PoseWithCovarianceArrayStamped
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose

def real_callback(real_data):
	get_model_position()
	# print "real_len = ", len(real_data.ranges)
	callback.__rscan = real_data

def fake_callback(fake_data):
	# print "fake_len = ", len(fake_data.ranges)
	callback.__fscan = fake_data

def get_model_position(name = 'evarobot'):
	model_pub = rospy.Publisher("model_pose", Pose)
	model_pose_msg = Pose()
	rospy.wait_for_service('/gazebo/get_model_state')
	try:
		_get_position = rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
		model_pose = _get_position(model_name=name)
		model_pose_msg = model_pose.pose
		model_pub.publish(model_pose_msg)
	except rospy.ServiceException, e:
		print "Service call failed:%s"%e

def callback(msgs):
	print "callback"
	f = open("lidar_compare.txt", 'a+')
	rscan = callback.__rscan
	fscan = callback.__fscan
	sum = 0
	print "r = %f  f = %f", len(rscan.ranges), len(fscan.ranges)
	for i in range(len(rscan.ranges)):
		delta = (rscan.ranges[i] - fscan.ranges[i])
		f.write(str(rscan.ranges[i]))
		f.write(' - ')
		f.write(str(fscan.ranges[i]))
		f.write(' = ')
		f.write(str(delta))
		f.write("\n")
		
		sum = sum + delta
	f.close()
	print "average = %f", sum/len(rscan.ranges)

def main():
	rospy.init_node('laser_compare')

	rospy.Subscriber("/compare_laser", Float32, callback)
	rospy.Subscriber("lidar", LaserScan, real_callback)
	rospy.Subscriber("train_fake_laser", LaserScan, fake_callback)
	rospy.spin()

if __name__ == '__main__':
	main()	