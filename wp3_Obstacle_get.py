#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 30 17:12:42 2021

@author: loki
"""

import time
import rospy
import tf
import numpy as np

from autoware_msgs.msg import Lane
from autoware_msgs.msg import 

def get_obstacles(msg):
	global Obstacles	
	received_obstacles = msg
	Obs_1=np.array([])
	
	f=open("wp3_obstacles_autoware.txt", "a+")
	f.write('Obstacle Coordinates: ' + str(len(received_obstacles)) + '\n')
	for i in range(len(received_waypoints.waypoints)):
		f.write(str(received_waypoints.waypoints[i].pose.pose.position) + '\n')
		path_wp_x = np.append(path_wp_x,received_waypoints.waypoints[i].pose.pose.position.x)
	print('waypoints received') 
	

if __name__ == '__main__':
	try:
				
		global path_wp_x,path_wp_y,path_wp_z
		path_wp_x = np.array([])
		path_wp_y = np.array([])
		path_wp_z = np.array([])
		rospy.init_node('wp3_waypoints', anonymous=True)
		waypoints_sub = rospy.Subscriber('/final_waypoints', Lane,waypoints_callback)
		while not rospy.is_shutdown(): #rate can be modified when needed
			if (len(path_wp_x) and len(path_wp_y) and len(path_wp_z))> 0 : 
				print('Entered if loop'+'\n')
				print('x: ' + str(path_wp_x) + '\n')
				print('y: ' + str(path_wp_y) + '\n')
				print('z: ' + str(path_wp_z) + '\n')
		rospy.spin()
			
	except rospy.ROSInterruptException:
		
		pass
