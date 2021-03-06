#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
import os
import message_filters
import time
from numpy import *
from pyflann import *
from sys import *
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from amcl.msg import PoseWithWeightArray
from amcl.msg import PoseWithCovarianceArrayStamped


global min_angle
global max_angle
global lidar_sample
global lidar_dict
global lidar_clust
global clust
global tao 
global hist_bin_size
global GSFH_database
global GSFH_RANGE
global Build_GSFH_Database
global flann
global line_threshold
global G_index
global Pose_table


min_angle = -135.0
max_angle = 135.0
lidar_sample = 360.0
lidar_range = 30.0
lidar_dict = {}
hist_bin_size = 5
range_hist_bin_size = 2
tao = 0.5   						# the threshold to classicfid lidar message
line_threshold = 1 				# the threshold to find the flexion
Build_GSFH_Database = False
Offline_Build = False
Online_Build = False
GSFH_Database_name = "GSFH_small_v4.dat" #"GSFH_Database_name_test"
GSFH_Database_name_range = "GSFH_Range_small_v4.dat"
Index_Table_name = "IndexTable_small_v4.dat"
G_index = 0


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
		theta = index_to_angle(clust_data[i])
		range_data = lidar_dict[clust_data[i]]
		tmp_coordinary.append( range_data * math.cos(math.radians(theta)) ) 
		tmp_coordinary.append( range_data * math.sin(math.radians(theta)) )
		tmp_coordinary.append( 0 ) # for point's normal vector k 
		coordinary_list.append(tmp_coordinary)
	coordinary_array = array(coordinary_list)

	for i in range(len(clust_pointlist) - 1):
		k,b = formed_line(clust_data[clust_pointlist[i]], clust_data[clust_pointlist[i + 1]])
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
		if isnan(alpha_angle):
			alpha_angle = 0
			print "NaN message1 ################################### "
		mix_hist[ int( alpha_angle/ hist_bin_size) ] = mix_hist[ int( alpha_angle/ hist_bin_size) ] + 1
		alpha_angle = alpha_funtion(coordinary_array_two[len(clust_data_two) - 1], coordinary_array_one[i] )
		if isnan(alpha_angle):
			alpha_angle = 0
			print "NaN message2 ################################### "
		mix_hist[ int( alpha_angle/ hist_bin_size) ] = mix_hist[ int( alpha_angle/ hist_bin_size) ] + 1

	for i in range(1, len(clust_data_two) - 1):
		alpha_angle = alpha_funtion(coordinary_array_one[0], coordinary_array_two[i] )
		if isnan(alpha_angle):
			alpha_angle = 0
			print "NaN message3 ################################### "
		mix_hist[ int( alpha_angle/ hist_bin_size) ] = mix_hist[ int( alpha_angle/ hist_bin_size) ] + 1
		alpha_angle = alpha_funtion(coordinary_array_one[len(clust_data_one) - 1], coordinary_array_two[i] )
		if isnan(alpha_angle):
			alpha_angle = 0
			print "NaN message4 ################################### "
		mix_hist[ int( alpha_angle/ hist_bin_size) ] = mix_hist[ int( alpha_angle/ hist_bin_size) ] + 1
	mix_hist = [int(mix_hist[x] / 4) for x in range(len(mix_hist))]
	return mix_hist

def get_GSFH(clust_data, clust_pointlist):
	hist = []
	for i in range(180/hist_bin_size):
		hist.append(0)

	coordinary_array = index_to_cartesian(clust_data, clust_pointlist)

	for i in range(1, len(clust_data) - 1):
		alpha_angle = alpha_funtion(coordinary_array[0], coordinary_array[i] )
		if isnan(alpha_angle):
			alpha_angle = 0
			print "NaN message5 ################################### "
		hist[ int( alpha_angle/ hist_bin_size) ] = hist[ int( alpha_angle/ hist_bin_size) ] + 1
		alpha_angle = alpha_funtion(coordinary_array[len(clust_data) - 1], coordinary_array[i] )
		if isnan(alpha_angle):
			alpha_angle = 0
			print "NaN message6 ################################### "
		hist[ int( alpha_angle/ hist_bin_size) ] = hist[ int( alpha_angle/ hist_bin_size) ] + 1
	hist = [int(hist[x]/2) for x in range(len(hist))]

	return hist

def formed_line(head_node, tail_node):
	x = []
	y = []

	theta = index_to_angle(head_node)
	range_data = lidar_dict[head_node]
	# theta = index_to_angle(lidar_dict, head_node)
	x.append( range_data * math.cos(math.radians(theta)) )
	y.append( range_data * math.sin(math.radians(theta)) )

	theta = index_to_angle(tail_node)
	range_data = lidar_dict[tail_node]
	# theta = index_to_angle(lidar_dict, tail_node)
	x.append( range_data * math.cos(math.radians(theta)) )
	y.append( range_data * math.sin(math.radians(theta)) )
	# print y[1], y[0], x[1], x[0]
	k = (y[1] - y[0] + 0.0000000001 )  / ((x[1] - x[0]) + 0.0000000001)
	b = y[1] - k * x[1]

	# print "formed k , b = " , k,b
	return k, b

def find_node(clust_data):
	inflexion = -1
	line_threshold = 1
	max_threshold = -1
	if len(clust_data) > 0:
		k, b = formed_line(clust_data[0], clust_data[ -1 ]) 
		for i in range(len(clust_data)):
			theta = index_to_angle(clust_data[i])
			range_data = lidar_dict[clust_data[i]]
			# theta = index_to_angle(lidar_dict, clust_data[i])
			x = range_data * math.cos(math.radians(theta))
			y = range_data * math.sin(math.radians(theta))
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
	else:
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

def index_to_angle(clust_index):
	return clust_index * (max_angle - min_angle) / lidar_sample + min_angle

def find_clust(list_data):
	first_max_clust = 0
	second_max_clust = 0
	data_tmp = []
	sort_tmp = []
	if len(list_data) <= 1:
		return 0, 0 
	for i in range(len(list_data)):
		data_tmp.append(len(list_data[i]))
		sort_tmp.append(len(list_data[i]))
	sort_tmp.sort()
	return data_tmp.index(sort_tmp[-1]), data_tmp.index(sort_tmp[-2])

def build_database(GSFH):
	f = open(GSFH_Database_name, 'a+')
	f.writelines(str(GSFH))
	f.write("\n")
	f.close()

def get_clustrange(clust_data):
	clust_range = []
	for i in range(int(lidar_range / range_hist_bin_size)):
		clust_range.append(0)

	for i in range(len(clust_data)):
		index = int((lidar_dict[clust_data[i]] - 0.001) / range_hist_bin_size)
		clust_range[index] = clust_range[index] + 1

	clust_range = [int(clust_range[x]/range_hist_bin_size) for x in range(len(clust_range))]

	return clust_range

def extract_features(data):
	lidar_clust = []
	clust = []

	for i in range(len(data.ranges)):
		lidar_dict[i] = data.ranges[i]
		if (i < len(data.ranges)-1) and (data.ranges[i + 1] - data.ranges[i] < tao) and (data.ranges[i + 1] - data.ranges[i] > -tao):
			clust.append(i)
		else :
			clust.append(i)
			lidar_clust.append(clust)
			clust = []
	# print "total len = ", len(lidar_clust)
	# for i in range(len(lidar_clust)):
	# 	print len(lidar_clust[i]),
	# print 

	Q1_index, Q2_index = find_clust(lidar_clust)  # find the largest and second largest clust
	Q1 = lidar_clust[Q1_index]
	Q2 = lidar_clust[Q2_index]	

	# print "Dealing with Q1"
	Q1_pointlist = line_fitting(Q1) # check if clust has inflexion
	Q1_hist = get_GSFH(Q1, Q1_pointlist)
	Q1_range = get_clustrange(Q1)
	# print "Dealing with Q2"
	Q2_pointlist = line_fitting(Q2)
	Q2_hist = get_GSFH(Q2, Q2_pointlist)
	Q2_range = get_clustrange(Q2)
	# print "Dealing with Mix_hist"
	Mix_hist = get_mix_GSFH(Q1, Q2, Q1_pointlist, Q2_pointlist)

	GSFH = Q1_hist + Q2_hist + Mix_hist
	My_GSFH = Q1_hist + Q2_hist + Mix_hist + Q1_range + Q2_range
	# print GSFH
	# print My_GSFH

	return GSFH, My_GSFH

	# if Build_GSFH_Database == True:
	# 	## build_database(GSFH)
	# 	print "Start to build!!"
	# else:
	# 	current_GSFH = array([float(x) for x in GSFH])
	# 	# result, dists = flann.nn(GSFH_database, current_GSFH, 4, algorithm="kmeans", branching=32, iterations=7, checks=16)
	# 	result, dists = flann.nn(GSFH_database, current_GSFH, 4)
	# 	print result	

def init_poses(possible_poses):
	poses = PoseWithCovarianceArrayStamped()
	poses.header.stamp = rospy.Time(0)
	poses.header.frame_id = "map"

	for i in range(len(possible_poses)):
		pose = PoseWithCovariance()
		pose.pose.position.x = possible_poses[i][0]
		pose.pose.position.y = possible_poses[i][1]
		pose.pose.position.z = 0
		pose.pose.orientation.w = possible_poses[i][2]
		pose.covariance = [0] * 36
		pose.covariance[0] = 0.2
		pose.covariance[7] = 0.2
		pose.covariance[35] = 0.1 * math.pi
		poses.pose_with_covariance_array.append(pose)
	mutil_pub = rospy.Publisher("initialposes", PoseWithCovarianceArrayStamped, queue_size=10)
	rate = rospy.Rate(1)
	while mutil_pub.get_num_connections() == 0:
		rate.sleep()
		continue
	mutil_pub.publish(poses)

def search_pose_result(possible_poses):
	pose_msg = PoseArray()
	single_pose_msg = Pose()
	result_pub = rospy.Publisher("search_pose", PoseArray)

	for i in range(len(possible_poses)):
		single_pose_msg.position.x = possible_poses[i][0]
		single_pose_msg.position.y = possible_poses[i][1]
		single_pose_msg.position.z = 0
		single_pose_msg.orientation.x = 0 
		single_pose_msg.orientation.y = 0
		single_pose_msg.orientation.z = possible_poses[i][2]
		single_pose_msg.orientation.w = possible_poses[i][3]
		pose_msg.poses.append(single_pose_msg)
		
	result_pub.publish(pose_msg)


def flann_search(base,data):
	current_GSFH = array([float(x) for x in data])
	start_time = time.time()
	result, dists = flann.nn(base, current_GSFH, 5)
	print "calc time = ", time.time() - start_time
	print result
	checktable(Pose_table, result)

def checktable(posetable, result):
	search_pose = []
	for i in range(len(result[0])):
		search_pose.append(posetable[result[0][i]])
		print posetable[result[0][i]]

	# search_pose_result(search_pose)
	# init_poses(search_pose)

def lidar_callback(data):
	rospy.loginfo(rospy.get_caller_id() + "Get lidar message")
	realtime_GSFH ,G= extract_features(data)
	print "paper result : "
	flann_search(GSFH_database,realtime_GSFH)
	print "my result :"
	flann_search(GSFH_RANGE,G)


def build_database(GSFH, file_name):
	f = open(file_name, 'a+')
	f.writelines(str(GSFH))
	f.write("\n")
	f.close()

def build_indextable(pose):
	f = open(Index_Table_name, 'a+')
	f.write(str(pose.position.x))
	f.write(' ')
	f.write(str(pose.position.y))
	f.write(' ')
	f.write(str(pose.orientation.z))
	f.write(' ')
	f.write(str(pose.orientation.w))
	f.write("\n")
	f.close()


def read_database(file_name):
	f = open(file_name,'r')
	data_base = []
	for line in f:
		data = line[1:len(line) - 2].split(',')
		data = [float(x) for x in data]
		data_base.append(data)

	return array(data_base)

def read_indextable():
	pose_table_dict = {}
	index = 0
	f = open(Index_Table_name, 'r')
	for line in f:
		data = line.strip().split(' ')
		data = [float(x) for x in data]
		pose_table_dict[index] = data
		index = index + 1

	return pose_table_dict


def relocalization():
	rospy.init_node('relocalization', anonymous = True)
	rospy.Subscriber("lidar", LaserScan, lidar_callback)
	rospy.spin()

def syc_callback(lidar_data, amcl_data):
	GSFH,My_GSFH = extract_features(lidar_data)
	if Build_GSFH_Database == True:
		build_database(GSFH, GSFH_Database_name)
		build_database(My_GSFH, GSFH_Database_name_range)
		build_indextable(amcl_data.pose.pose)

	rospy.loginfo(rospy.get_caller_id() + "Building Database...")

def trainPoses_callback(pose_data):
	if Build_GSFH_Database == True:
		# print "Building indextable ..."
		build_indextable(pose_data)

def trainGSFH_callback(lidar_data):
	GSFH,My_GSFH = extract_features(lidar_data)
	if Build_GSFH_Database == True:
		# print "Building database ..."
		build_database(GSFH, GSFH_Database_name)
		build_database(My_GSFH, GSFH_Database_name_range)


if __name__ == '__main__':
	if len(argv) >= 3 and argv[1] == "--build":
		input = raw_input("Sure to rebuild the total database? ( Y/N )")
		if input == "Y" or input == "y":
			print "Start rebuild process ..."
			if os.path.exists(GSFH_Database_name):
				print "Delet GSFH_Database"
				os.remove(GSFH_Database_name)
			if os.path.exists(Index_Table_name):
				print "Delet IndexTable"
				os.remove(Index_Table_name)
			if os.path.exists(GSFH_Database_name_range):
				print "Delet GSFH_Database_name_range"
				os.remove(GSFH_Database_name_range)
			Build_GSFH_Database = True
		else:
			print "Give up rebuild process ..."
			exit(0)

		if argv[2] == "--offline":
			Online_Build = False
			Offline_Build = True
		elif argv[2] == "--online":
			Online_Build = True
			Offline_Build = False
		else:
			print "Online or Offline build database?"

	else:
		GSFH_database = read_database(GSFH_Database_name)
		GSFH_RANGE = read_database(GSFH_Database_name_range)
		Pose_table = read_indextable()
		flann = FLANN()
	rospy.init_node('relocalization', anonymous = True)
	# rospy.Subscriber("lidar", LaserScan, lidar_callback)
	# rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amcl_callback)
	if Build_GSFH_Database == True:
		if Offline_Build == True:
			print "Start Offline Building ..."
			rospy.Subscriber("train_poses", Pose, trainPoses_callback)
			rospy.Subscriber("train_fake_laser", LaserScan, trainGSFH_callback)
		if Online_Build == True:
			print "Start Online Building ..."
			lidar_sub = message_filters.Subscriber('lidar', LaserScan)
			amcl_sub = message_filters.Subscriber('amcl_pose', PoseWithCovarianceStamped)
			ts = message_filters.TimeSynchronizer([lidar_sub, amcl_sub], 10)
			ts.registerCallback(syc_callback)
	else:
		rospy.Subscriber("lidar", LaserScan, lidar_callback)
	rospy.spin()
			