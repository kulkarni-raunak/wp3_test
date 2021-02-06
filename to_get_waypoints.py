#!/usr/bin/env python

import time
import rospy
import tf
import numpy as np

from autoware_msgs.msg import Lane

def waypoints_callback(msg):
	global path_wp_x,path_wp_y,path_wp_z	
	received_waypoints = msg
	path_wp_x=np.array([])
	path_wp_y=np.array([])
	path_wp_z=np.array([])
	f=open("testing_waypoints.txt", "a+")
	f.write('No. of Waypoints received: ' + str(len(received_waypoints.waypoints)) + '\n')
	for i in range(len(received_waypoints.waypoints)):
		f.write(str(received_waypoints.waypoints[i].pose.pose.position) + '\n')
		path_wp_x = np.append(path_wp_x,received_waypoints.waypoints[i].pose.pose.position.x)
		path_wp_y = np.append(path_wp_y,received_waypoints.waypoints[i].pose.pose.position.y)
		path_wp_z = np.append(path_wp_z,received_waypoints.waypoints[i].pose.pose.position.z)
	print('waypoints received') 
	

if __name__ == '__main__':
	try:
				
		global path_wp_x,path_wp_y,path_wp_z
		path_wp_x = np.array([])
		path_wp_y = np.array([])
		path_wp_z = np.array([])
		rospy.init_node('wp3_waypoints', anonymous=True)
		waypoints_sub = rospy.Subscriber('/base_waypoints', Lane,waypoints_callback)
		while not rospy.is_shutdown(): #rate can be modified when needed
			if (len(path_wp_x) and len(path_wp_y) and len(path_wp_z))> 0 : 
				print('Entered if loop'+'\n')
				print('x: ' + str(path_wp_x) + '\n')
				print('y: ' + str(path_wp_y) + '\n')
				print('z: ' + str(path_wp_z) + '\n')
		rospy.spin()
			
	except rospy.ROSInterruptException:
		
		pass
