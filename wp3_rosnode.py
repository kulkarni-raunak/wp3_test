#!/usr/bin/env python

import time
import rospy
import tf
import numpy as np


from carla_msgs.msg import CarlaEgoVehicleControl



if __name__ == '__main__':
	try:
	
		rospy.init_node('wp3_ego_vehicle_ctrl_cmd', anonymous=True)
		time.sleep(1)
		wp3_ego_vehicle_ctrl_cmd = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl,queue_size =10)
		time.sleep(1)
		msg = CarlaEgoVehicleControl()
		msg.throttle = 1.0
		msg.steer = 0.0
		rate = rospy.Rate(10) # 10hz

		while not rospy.is_shutdown():
			wp3_ego_vehicle_ctrl_cmd.publish(msg)
		#waypoint_pub.publish("TEST")
			print('publishing to ego_vehicle '+ str(msg))
			#time.sleep(0.2)
			rate.sleep()
			
	except rospy.ROSInterruptException:
		
		pass
