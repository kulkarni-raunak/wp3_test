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
import rospy
import tf
from nav_msgs.msg import Odometry
from derived_object_msgs.msg import ObjectArray
import gjk
from autoware_msgs.msg import CloudClusterArray
#

def get_rect(x, y, yaw, vehicleSize):

    dy = np.tan(yaw )
    
    headings = np.array([[1,dy]])
    headings = headings/(np.sqrt(1+dy**2))
  
    normVec = np.full_like(headings,1)
    normVec[0,0] = -1*headings[0,1]
    normVec[0,1] = headings[0,0]

    locn = np.array([[x, y]])
        
    topleft = locn + normVec[:,0:2] * 0.5* vehicleSize[1] - headings[:,0:2] * 0.5 * vehicleSize[0] 
    bottomleft = locn - normVec[:,0:2] * 0.5* vehicleSize[1] - headings[:,0:2] * 0.5 * vehicleSize[0] 
    topright = topleft + headings[:,0:2] * vehicleSize[0]
    bottomright = bottomleft + headings[:,0:2] * vehicleSize[0]

    return [topleft, bottomleft, bottomright, topright]

















