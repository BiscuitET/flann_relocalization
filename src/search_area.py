#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from numpy import *
from sensor_msgs.msg import LaserScan
from pyflann import *

class Flann_Class():

	def __init__(self):
		self.data_base = []
		self.current_data = []
		self.subscriber = rospy.Subscriber("lidar", LaserScan, self.callback, queue_size = 5)
		self.receive_data = False

	def load_dataset(self):
		data_file = open("map_360.dat", 'r')
		for line in data_file:
			source_data = line.strip()
			self.data_base.append([float(x) for x in source_data.split(' ')])
		self.data_base = array(self.data_base)
		print "Build Database Finish ! "
		print self.data_base.dtype

	def callback(self, data):
		self.current_data = [x for x in data.ranges]
		self.current_data = array(self.current_data)
		self.receive_data = True

	def search(self):
		#
		if self.receive_data == True:
			flann = FLANN()
			result, dists = flann.nn(self.data_base, self.current_data, 4, algorithm="kmeans", branching=32, iterations=7, checks=16)
			print "search result : " + str(result)
		else:
			print "No lidar data,Can't Find any result"

# def load_dataset():
# 	data_file = open("map.dat", 'r')
# 	data_base = []
# 	for line in data_file:
# 		source_data = line.strip()
# 		data_base.append([float(x) for x in source_data.split(' ')])
# 	data_base = array(data_base)

# def call_back(data):
# 	current_data = []
# 	for i in range(len(data.ranges)):
# 		current_data.append(data.ranges[i])
# 	current_data = array(current_data)
# 	print current_data.dtype

if __name__ == '__main__':
	my_flann = Flann_Class()
	my_flann.load_dataset()
	rospy.init_node('search_node', anonymous = True)
	while not rospy.is_shutdown():
		my_flann.search()
	exit(0)