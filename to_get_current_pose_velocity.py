#!/usr/bin/env python
#checking my git working 
import time
import rospy
import tf
import numpy as np

from geometry_msgs.msg import PoseStamped

def current_pose_callback(msg):	
	received_pose = msg
	x = msg.pose.position.x
	y = -(msg.pose.position.y)
	z = msg.pose.position.z
	quat = ( msg.pose.orientation.x, msg.pose.orientation.x, msg.pose.orientation.x, msg.pose.orientation.x)
	eul = tf.transformations.euler_from_quaternion(quat)
	roll = eul[0]
	pitch = eul[1]
	yaw = -1*eul[2]
	print('current pose: x			y		z		roll		pitch		yaw')
	print('	     '+str(x)+' '+str(y)+'	'+str(z)+'	 '+str(roll)+' 		'+str(pitch)+'		 '+str(yaw)+' \n')


if __name__ == '__main__':
	try:
		rospy.init_node('wp3_current_pose_velocity', anonymous=True)
		pose_sub = rospy.Subscriber('/current_pose',PoseStamped,current_pose_callback)
		#while not rospy.is_shutdown(): #rate can be modified when needed
			#if (len(path_wp_x) and len(path_wp_y) and len(path_wp_z))> 0 : 
				
		rospy.spin()
			
	except rospy.ROSInterruptException:
		
		pass
