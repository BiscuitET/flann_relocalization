#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
from numpy import *
from pyflann import *
from sys import *
from sensor_msgs.msg import LaserScan

global min_angle
global max_angle
global lidar_sample
global lidar_dict
global lidar_clust
global clust
global tao 
global hist_bin_size
global GSFH_database
global Build_GSFH_Database
global flann


min_angle = -135.0
max_angle = 135.0
lidar_sample = 360.0
lidar_dict = {}
hist_bin_size = 5
tao = 2
Build_GSFH_Database = False

def alpha_funtion(first_vector, second_vector):
	k1 = (second_vector[1] - first_vector[1])/(second_vector[0] - first_vector[0])
	k2 = second_vector[2]
	angle = arctan(abs((k2 - k1)/(1 + k1*k2))) * 180 / pi  # calc two lines angle
	if angle < 0:
		angle = angle + 180
	# print "angle = ", angle
	return angle

def index_to_cartesian(clust_data, clust_point_list):
	"""Turn lidar message [range, theta] to cartesian coordinates, and calc every point's normal vector slope"""
	clust_pointlist = clust_point_list[:]
	clust_pointlist.insert(0, 0)
	clust_pointlist.insert(len(clust_data), len(clust_data) - 1)
	coordinary_list = []
	for i in range(len(clust_data)):
		tmp_coordinary = []
		theta = index_to_angle(lidar_dict, clust_data[i])
		tmp_coordinary.append( clust_data[i] * math.cos(math.radians(theta)) ) 
		tmp_coordinary.append( clust_data[i] * math.sin(math.radians(theta)) )
		tmp_coordinary.append( 0 ) # for point's normal vector k 
		coordinary_list.append(tmp_coordinary)
	coordinary_array = array(coordinary_list)

	for i in range(len(clust_pointlist) - 1):
		k,b = formed_line(clust_data[clust_pointlist[i]], clust_data[clust_pointlist[i + 1]])
		# print "got k = " , k
		for j in range(clust_pointlist[i] ,clust_pointlist[i + 1]):
			coordinary_array[j][2] = -1 / k

	return coordinary_array

def get_mix_GSFH(clust_data_one, clust_data_two, clust_pointlist_one, clust_pointlist_two):
	mix_hist = []
	for i in range(180 / hist_bin_size):
		mix_hist.append(0)
	
	coordinary_array_one = index_to_cartesian(clust_data_one, clust_pointlist_one)
	coordinary_array_two = index_to_cartesian(clust_data_two, clust_pointlist_two)

	for i in range(1, len(clust_data_one) - 1):
		alpha_angle = alpha_funtion(coordinary_array_two[0], coordinary_array_one[i] )
		mix_hist[ int( alpha_angle/ hist_bin_size) ] = mix_hist[ int( alpha_angle/ hist_bin_size) ] + 1
		alpha_angle = alpha_funtion(coordinary_array_two[len(clust_data_two) - 1], coordinary_array_one[i] )
		mix_hist[ int( alpha_angle/ hist_bin_size) ] = mix_hist[ int( alpha_angle/ hist_bin_size) ] + 1

	for i in range(1, len(clust_data_two) - 1):
		alpha_angle = alpha_funtion(coordinary_array_one[0], coordinary_array_two[i] )
		mix_hist[ int( alpha_angle/ hist_bin_size) ] = mix_hist[ int( alpha_angle/ hist_bin_size) ] + 1
		alpha_angle = alpha_funtion(coordinary_array_one[len(clust_data_one) - 1], coordinary_array_two[i] )
		mix_hist[ int( alpha_angle/ hist_bin_size) ] = mix_hist[ int( alpha_angle/ hist_bin_size) ] + 1
	
	return mix_hist

def get_GSFH(clust_data, clust_pointlist):
	hist = []
	for i in range(180/hist_bin_size):
		hist.append(0)

	coordinary_array = index_to_cartesian(clust_data, clust_pointlist)

	for i in range(1, len(clust_data) - 1):
		alpha_angle = alpha_funtion(coordinary_array[0], coordinary_array[i] )
		hist[ int( alpha_angle/ hist_bin_size) ] = hist[ int( alpha_angle/ hist_bin_size) ] + 1
		alpha_angle = alpha_funtion(coordinary_array[len(clust_data) - 1], coordinary_array[i] )
		hist[ int( alpha_angle/ hist_bin_size) ] = hist[ int( alpha_angle/ hist_bin_size) ] + 1

	return hist

def formed_line(head_node, tail_node):
	x = []
	y = []
	theta = index_to_angle(lidar_dict, head_node)
	x.append( head_node * math.cos(math.radians(theta)) )
	y.append( head_node * math.sin(math.radians(theta)) )

	theta = index_to_angle(lidar_dict, tail_node)
	x.append( tail_node * math.cos(math.radians(theta)) )
	y.append( tail_node * math.sin(math.radians(theta)) )

	k = (y[1] - y[0]) / ((x[1] - x[0]) + 0.0000000001)
	b = y[1] - k * x[1]

	# print "formed k , b = " , k,b
	return k, b

def find_node(clust_data):
	inflexion = -1
	line_threshold = 3
	max_threshold = -1
	k, b = formed_line(clust_data[0], clust_data[ -1 ]) 
	for i in range(len(clust_data)):
		theta = index_to_angle(lidar_dict, clust_data[i])
		x = clust_data[i] * math.cos(math.radians(theta))
		y = clust_data[i] * math.sin(math.radians(theta))
		# tmp_threshold = abs(abs(k * x + b) - abs(clust_dict[x]))
		tmp_threshold = abs((k * x - 1 * y + b) / sqrt(k*k + 1))  # calc point to line distance
		if (tmp_threshold < line_threshold) :
			pass
		else :
			if tmp_threshold > max_threshold :
				max_threshold = tmp_threshold
				inflexion = i
	# print "max_threshold = ", max_threshold
	return inflexion


def myself_line_fitting(clust_data):
	test = open("coordinary.dat", 'w+')
	print len(clust_data)
	line_threshold = 4
	clust_dict = {}
	max_threshold = 0
	inflexion = -1

	k, b = formed_line(clust_data[0], clust_data[-1])
	for i in range(len(clust_data)):
		theta = index_to_angle(lidar_dict, clust_data[i])
		x = clust_data[i] * math.cos(math.radians(theta))
		y = clust_data[i] * math.sin(math.radians(theta))
		clust_dict[x] = y
		tmp_threshold = abs(abs(k * x + b) - abs(clust_dict[x]))
		if (tmp_threshold < line_threshold) :
			pass
		else :
			if tmp_threshold > max_threshold :
				max_threshold = tmp_threshold
				inflexion = i

		test.write(str(x))
		test.write(" , ")
		test.write(str(y))
		test.write("\n")
	test.close()	
	# print "inflexion = ", inflexion
	if inflexion != -1:
		line_fitting(clust_data[inflexion : ])
	# return inflexion	

def line_fitting(clust_data):
	tail_node = len(clust_data) - 1
	head_node = 0
	point_list = []
	while True:
		tmp_node = find_node(clust_data[head_node:tail_node])
		if tmp_node == -1:
			break
		while True:
			tail_node = head_node + tmp_node
			tmp_node = find_node(clust_data[head_node:tail_node])
			if tmp_node == -1 :
				break
		head_node = tail_node
		tail_node = len(clust_data) - 1
		# print "Got a point = ", head_node
		point_list.append(head_node)
	return point_list

def index_to_angle(dict_data,clust_index):
	return dict_data[clust_index] * (max_angle - min_angle) / lidar_sample + min_angle

def find_clust(list_data):
	first_max_clust = 0
	second_max_clust = 0
	data_tmp = []
	sort_tmp = []
	if len(list_data) == 1:
		return 0, 0 
	for i in range(len(list_data)):
		data_tmp.append(len(list_data[i]))
		sort_tmp.append(len(list_data[i]))
	sort_tmp.sort()
	return data_tmp.index(sort_tmp[-1]), data_tmp.index(sort_tmp[-2])

def build_database(GSFH):
	f = open("GSFH.dat", 'a+')
	f.writelines(str(GSFH))
	f.write("\n")
	f.close()

def callback(data):
	lidar_clust = []
	clust = []
	rospy.loginfo(rospy.get_caller_id() + "Get lidar message")
	for i in range(len(data.ranges)):
		lidar_dict[data.ranges[i]] = i
		if (i < len(data.ranges)-1) and (data.ranges[i + 1] - data.ranges[i] < tao) and (data.ranges[i + 1] - data.ranges[i] > -tao):
			clust.append(data.ranges[i])
		else :
			clust.append(data.ranges[i])
			lidar_clust.append(clust)
			clust = []
	print "total len = ", len(lidar_clust)
	for i in range(len(lidar_clust)):
		print len(lidar_clust[i]),
	print 

	Q1_index, Q2_index = find_clust(lidar_clust)  # find the largest and second largest clust
	Q1 = lidar_clust[Q1_index]
	Q2 = lidar_clust[Q2_index]
	Q1_pointlist = line_fitting(Q1) # check if clust has inflexion
	Q2_pointlist = line_fitting(Q2)

	Q1_hist = get_GSFH(Q1, Q1_pointlist)
	Q2_hist = get_GSFH(Q2, Q2_pointlist)
	Mix_hist = get_mix_GSFH(Q1, Q2, Q1_pointlist, Q2_pointlist)

	GSFH = Q1_hist + Q2_hist + Mix_hist
	current_GSFH = array([float(x) for x in GSFH])

	result, dists = flann.nn(GSFH_database, current_GSFH, 4, algorithm="kmeans", branching=32, iterations=7, checks=16)
	print result
	if Build_GSFH_Database == True:
		print "Start to build!!"

def read_database():
	f = open("GSFH.dat",'r')
	data_base = []
	for line in f:
		data = line[1:len(line) - 2].split(',')
		data = [float(x) for x in data]
		data_base.append(data)

	return array(data_base)

def listener():
	rospy.init_node('listener', anonymous = True)
	rospy.Subscriber("lidar", LaserScan, callback)
	rospy.spin()

if __name__ == '__main__':
	if len(argv) >= 2 and argv[1] == "-build":
		Build_GSFH_Database = True
	GSFH_database = read_database()
	flann = FLANN()
	rospy.init_node('listener', anonymous = True)
	rospy.Subscriber("lidar", LaserScan, callback)
	rospy.spin()
	# while not rospy.is_shutdown():
	# 	if receive_data == True:
	# 		result, dists = flann.nn(GSFH_database, current_GSFH, 4, algorithm="kmeans", branching=32, iterations=7, checks=16)
	# 		print result
	# exit(0)
			