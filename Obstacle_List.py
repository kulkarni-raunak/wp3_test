#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Feb  6 16:19:23 2021

@author: loki
"""
import time
import rospy
import tf
import numpy as np

from autoware_msgs.msg import DetectedObjectArray

def ObstacleList_callback(msg):
	global obstacles_list	
	received_obstacleslist = msg
	f=open("Obstacles_WP3.txt", "a+")
	f.write('No. of Objects received: ' + str(len(received_obstacleslist.objects)) + '\n')
	#print('------------------New One-----------------------') 

if __name__ == '__main__':
	try:
				
		
		rospy.init_node('wp3_ObstaclesList', anonymous=True)
		waypoints_sub = rospy.Subscriber('/prediction/motion_predictor/objects', DetectedObjectArray,ObstacleList_callback)
		#while not rospy.is_shutdown(): #rate can be modified when needed
			#time.sleep(0.2)
		#rospy.spin()
			
	except rospy.ROSInterruptException:
		
		pass



