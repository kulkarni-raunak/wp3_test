
import roslaunch
import rospy
import os
import pymsgbox
import matplotlib.pyplot as plt




# 3 seconds later
# launch.shutdown()

A = pymsgbox.alert("Launch python working..",title="WP3 Msg")

if A=='OK':
    print('Python File invoked...')
os.system("roslaunch carla_autoware_agent carla_autoware_agent.launch town:=Town03")    
