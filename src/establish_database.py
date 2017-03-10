#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "Get lidar message %s",data.ranges)
	file = open("map_360.dat", 'a+')
	for i in range(len(data.ranges)):
		file.write(str(data.ranges[i]))
		file.write(' ')
	file.write('\n')

def listener():
	rospy.init_node('listener', anonymous = True)
	rospy.Subscriber("lidar", LaserScan, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
