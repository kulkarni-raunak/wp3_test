#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb 14 16:49:25 2021

@author: lfzt3
"""

import roslaunch
import rospy

rospy.init_node('WP3_Autoware', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/autoware/carla-autoware/carla-autoware-agent/launch/carla_autoware_agent.launch"])
launch.start()
rospy.loginfo("started")

rospy.sleep(3)
# 3 seconds later
# launch.shutdown()
