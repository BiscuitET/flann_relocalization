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


# 	============
# 	#!/usr/bin/python
# # -*- coding: utf-8 -*-

# import rospy
# import math
# import os
# from numpy import *
# from pyflann import *
# from sys import *
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import PoseWithCovarianceStamped
# from amcl.msg import PoseWithWeightArray
# from amcl.msg import PoseWithCovarianceArrayStamped

# global min_angle
# global max_angle
# global lidar_sample
# global lidar_dict
# global lidar_clust
# global clust
# global tao 
# global hist_bin_size
# global GSFH_database
# global Build_GSFH_Database
# global flann
# global line_threshold


# min_angle = -135.0
# max_angle = 135.0
# lidar_sample = 360.0
# lidar_dict = {}
# hist_bin_size = 5
# tao = 2   						# the threshold to classicfid lidar message
# line_threshold = 1 				# the threshold to find the flexion
# Build_GSFH_Database = False
# GSFH_Database_name = "GSFH.dat"

# def alpha_funtion(first_vector, second_vector):
# 	k1 = (second_vector[1] - first_vector[1])/(second_vector[0] - first_vector[0])
# 	k2 = second_vector[2]
# 	angle = arctan(abs((k2 - k1)/(1 + k1*k2))) * 180 / pi  # calc two lines angle
# 	if angle < 0:
# 		angle = angle + 180
# 	# print "angle = ", angle
# 	return angle

# def index_to_cartesian(clust_data, clust_point_list):
# 	"""Turn lidar message [range, theta] to cartesian coordinates, and calc every point's normal vector slope"""
# 	clust_pointlist = clust_point_list[:]
# 	clust_pointlist.insert(0, 0)
# 	clust_pointlist.insert(len(clust_data), len(clust_data) - 1)
# 	coordinary_list = []
# 	for i in range(len(clust_data)):
# 		tmp_coordinary = []
# 		theta = index_to_angle(clust_data[i])
# 		range_data = lidar_dict[clust_data[i]]
# 		tmp_coordinary.append( range_data * math.cos(math.radians(theta)) ) 
# 		tmp_coordinary.append( range_data * math.sin(math.radians(theta)) )
# 		tmp_coordinary.append( 0 ) # for point's normal vector k 
# 		coordinary_list.append(tmp_coordinary)
# 	coordinary_array = array(coordinary_list)

# 	for i in range(len(clust_pointlist) - 1):
# 		k,b = formed_line(clust_data[clust_pointlist[i]], clust_data[clust_pointlist[i + 1]])
# 		for j in range(clust_pointlist[i] ,clust_pointlist[i + 1]):
# 			coordinary_array[j][2] = -1 / k

# 	return coordinary_array

# def get_mix_GSFH(clust_data_one, clust_data_two, clust_pointlist_one, clust_pointlist_two):
# 	mix_hist = []
# 	for i in range(180 / hist_bin_size):
# 		mix_hist.append(0)
	
# 	coordinary_array_one = index_to_cartesian(clust_data_one, clust_pointlist_one)
# 	coordinary_array_two = index_to_cartesian(clust_data_two, clust_pointlist_two)

# 	for i in range(1, len(clust_data_one) - 1):
# 		alpha_angle = alpha_funtion(coordinary_array_two[0], coordinary_array_one[i] )
# 		if isnan(alpha_angle):
# 			print "NaN message ################################### "
# 		mix_hist[ int( alpha_angle/ hist_bin_size) ] = mix_hist[ int( alpha_angle/ hist_bin_size) ] + 1
# 		alpha_angle = alpha_funtion(coordinary_array_two[len(clust_data_two) - 1], coordinary_array_one[i] )
# 		mix_hist[ int( alpha_angle/ hist_bin_size) ] = mix_hist[ int( alpha_angle/ hist_bin_size) ] + 1

# 	for i in range(1, len(clust_data_two) - 1):
# 		alpha_angle = alpha_funtion(coordinary_array_one[0], coordinary_array_two[i] )
# 		mix_hist[ int( alpha_angle/ hist_bin_size) ] = mix_hist[ int( alpha_angle/ hist_bin_size) ] + 1
# 		alpha_angle = alpha_funtion(coordinary_array_one[len(clust_data_one) - 1], coordinary_array_two[i] )
# 		mix_hist[ int( alpha_angle/ hist_bin_size) ] = mix_hist[ int( alpha_angle/ hist_bin_size) ] + 1
	
# 	return mix_hist

# def get_GSFH(clust_data, clust_pointlist):
# 	hist = []
# 	for i in range(180/hist_bin_size):
# 		hist.append(0)

# 	coordinary_array = index_to_cartesian(clust_data, clust_pointlist)

# 	for i in range(1, len(clust_data) - 1):
# 		alpha_angle = alpha_funtion(coordinary_array[0], coordinary_array[i] )
# 		hist[ int( alpha_angle/ hist_bin_size) ] = hist[ int( alpha_angle/ hist_bin_size) ] + 1
# 		alpha_angle = alpha_funtion(coordinary_array[len(clust_data) - 1], coordinary_array[i] )
# 		hist[ int( alpha_angle/ hist_bin_size) ] = hist[ int( alpha_angle/ hist_bin_size) ] + 1

# 	return hist

# def formed_line(head_node, tail_node):
# 	x = []
# 	y = []

# 	theta = index_to_angle(head_node)
# 	range_data = lidar_dict[head_node]
# 	# theta = index_to_angle(lidar_dict, head_node)
# 	x.append( range_data * math.cos(math.radians(theta)) )
# 	y.append( range_data * math.sin(math.radians(theta)) )

# 	theta = index_to_angle(tail_node)
# 	range_data = lidar_dict[tail_node]
# 	# theta = index_to_angle(lidar_dict, tail_node)
# 	x.append( range_data * math.cos(math.radians(theta)) )
# 	y.append( range_data * math.sin(math.radians(theta)) )
# 	# print y[1], y[0], x[1], x[0]
# 	k = (y[1] - y[0]) / ((x[1] - x[0]) + 0.0000000001)
# 	b = y[1] - k * x[1]

# 	# print "formed k , b = " , k,b
# 	return k, b

# def find_node(clust_data):
# 	inflexion = -1
# 	line_threshold = 1
# 	max_threshold = -1
# 	k, b = formed_line(clust_data[0], clust_data[ -1 ]) 
# 	for i in range(len(clust_data)):
# 		theta = index_to_angle(clust_data[i])
# 		range_data = lidar_dict[clust_data[i]]
# 		# theta = index_to_angle(lidar_dict, clust_data[i])
# 		x = range_data * math.cos(math.radians(theta))
# 		y = range_data * math.sin(math.radians(theta))
# 		# tmp_threshold = abs(abs(k * x + b) - abs(clust_dict[x]))
# 		tmp_threshold = abs((k * x - 1 * y + b) / sqrt(k*k + 1))  # calc point to line distance
# 		if (tmp_threshold < line_threshold) :
# 			pass
# 		else :
# 			if tmp_threshold > max_threshold :
# 				max_threshold = tmp_threshold
# 				inflexion = i
# 	# print "max_threshold = ", max_threshold
# 	return inflexion


# def myself_line_fitting(clust_data):
# 	test = open("coordinary.dat", 'w+')
# 	print len(clust_data)
# 	line_threshold = 4
# 	clust_dict = {}
# 	max_threshold = 0
# 	inflexion = -1

# 	k, b = formed_line(clust_data[0], clust_data[-1])
# 	for i in range(len(clust_data)):
# 		theta = index_to_angle(lidar_dict, clust_data[i])
# 		x = clust_data[i] * math.cos(math.radians(theta))
# 		y = clust_data[i] * math.sin(math.radians(theta))
# 		clust_dict[x] = y
# 		tmp_threshold = abs(abs(k * x + b) - abs(clust_dict[x]))
# 		if (tmp_threshold < line_threshold) :
# 			pass
# 		else :
# 			if tmp_threshold > max_threshold :
# 				max_threshold = tmp_threshold
# 				inflexion = i

# 		test.write(str(x))
# 		test.write(" , ")
# 		test.write(str(y))
# 		test.write("\n")
# 	test.close()	
# 	# print "inflexion = ", inflexion
# 	if inflexion != -1:
# 		line_fitting(clust_data[inflexion : ])
# 	# return inflexion	

# def line_fitting(clust_data):
# 	tail_node = len(clust_data) - 1
# 	head_node = 0
# 	point_list = []
# 	while True:
# 		tmp_node = find_node(clust_data[head_node:tail_node])
# 		if tmp_node == -1:
# 			break
# 		while True:
# 			tail_node = head_node + tmp_node
# 			tmp_node = find_node(clust_data[head_node:tail_node])
# 			if tmp_node == -1 :
# 				break
# 		head_node = tail_node
# 		tail_node = len(clust_data) - 1
# 		print "Got a point = ", head_node
# 		point_list.append(head_node)
# 	return point_list

# def index_to_angle(clust_index):
# 	return clust_index * (max_angle - min_angle) / lidar_sample + min_angle

# def find_clust(list_data):
# 	first_max_clust = 0
# 	second_max_clust = 0
# 	data_tmp = []
# 	sort_tmp = []
# 	if len(list_data) == 1:
# 		return 0, 0 
# 	for i in range(len(list_data)):
# 		data_tmp.append(len(list_data[i]))
# 		sort_tmp.append(len(list_data[i]))
# 	sort_tmp.sort()
# 	return data_tmp.index(sort_tmp[-1]), data_tmp.index(sort_tmp[-2])

# def build_database(GSFH):
# 	f = open(GSFH_Database_name, 'a+')
# 	f.writelines(str(GSFH))
# 	f.write("\n")
# 	f.close()

# def lidar_callback(data):
# 	lidar_clust = []
# 	clust = []
# 	rospy.loginfo(rospy.get_caller_id() + "Get lidar message")
# 	for i in range(len(data.ranges)):
# 		lidar_dict[i] = data.ranges[i]
# 		if (i < len(data.ranges)-1) and (data.ranges[i + 1] - data.ranges[i] < tao) and (data.ranges[i + 1] - data.ranges[i] > -tao):
# 			clust.append(i)
# 		else :
# 			clust.append(i)
# 			lidar_clust.append(clust)
# 			clust = []
# 	print "total len = ", len(lidar_clust)
# 	for i in range(len(lidar_clust)):
# 		print len(lidar_clust[i]),
# 	print 

# 	Q1_index, Q2_index = find_clust(lidar_clust)  # find the largest and second largest clust
# 	Q1 = lidar_clust[Q1_index]
# 	Q2 = lidar_clust[Q2_index]	

# 	print "Dealing with Q1"
# 	Q1_pointlist = line_fitting(Q1) # check if clust has inflexion
# 	Q1_hist = get_GSFH(Q1, Q1_pointlist)
# 	print "Dealing with Q2"
# 	Q2_pointlist = line_fitting(Q2)
# 	Q2_hist = get_GSFH(Q2, Q2_pointlist)
# 	print "Dealing with Mix_hist"
# 	Mix_hist = get_mix_GSFH(Q1, Q2, Q1_pointlist, Q2_pointlist)

# 	GSFH = Q1_hist + Q2_hist + Mix_hist

# 	if Build_GSFH_Database == True:
# 		## build_database(GSFH)
# 		print "Start to build!!"
# 	else:
# 		current_GSFH = array([float(x) for x in GSFH])
# 		# result, dists = flann.nn(GSFH_database, current_GSFH, 4, algorithm="kmeans", branching=32, iterations=7, checks=16)
# 		result, dists = flann.nn(GSFH_database, current_GSFH, 4)
# 		print result

# def amcl_callback(data):
# 	print "receive amcl_posedata!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	

# def build_database(GSFH):
# 	f = open(GSFH_Database_name, 'a+')
# 	f.writelines(str(GSFH))
# 	f.write("\n")
# 	f.close()

# def read_database():
# 	f = open(GSFH_Database_name,'r')
# 	data_base = []
# 	for line in f:
# 		data = line[1:len(line) - 2].split(',')
# 		data = [float(x) for x in data]
# 		data_base.append(data)

# 	return array(data_base)

# def relocalization():
# 	rospy.init_node('relocalization', anonymous = True)
# 	rospy.Subscriber("lidar", LaserScan, lidar_callback)
# 	rospy.spin()

# if __name__ == '__main__':
# 	if len(argv) >= 2 and argv[1] == "-build":
# 		input = raw_input("Sure to rebuild the total database? ( Y/N )")
# 		if input == "Y" or input == "y":
# 			print "Start rebuild process ..."
# 			if os.path.exists(GSFH_Database_name):
# 				print "Delet file"
# 				# os.remove(GSFH_Database_name)
# 			Build_GSFH_Database = True
# 		else:
# 			print "Give up rebuild process ..."
# 			exit(0)
# 	else:
# 		GSFH_database = read_database()
# 		flann = FLANN()
# 	rospy.init_node('relocalization', anonymous = True)
# 	rospy.Subscriber("lidar", LaserScan, lidar_callback)
# 	rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amcl_callback)
# 	rospy.spin()
# 	# while not rospy.is_shutdown():
# 	# 	if receive_data == True:
# 	# 		result, dists = flann.nn(GSFH_database, current_GSFH, 4, algorithm="kmeans", branching=32, iterations=7, checks=16)
# 	# 		print result
# 	# exit(0)
# 			