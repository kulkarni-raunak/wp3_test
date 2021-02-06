import rospy
import tf
from nav_msgs.msg import Odometry
from derived_object_msgs.msg import ObjectArray
import numpy as np
import gjk

obstacle_sub_list = []
vehicle_id_prev = []


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


class Obstacles:

    # Members
    #
    #   obstacle_data_list:     List to store obstacle data
    #   vehicle_id_prev:        List of vehicles in previous cycle

    
    def __init__(self):
        
        self.obstacle_data_list = []
        self.vehicle_id_prev = []
        self.spline = None
        self.obstacle = []


    def update_obstacle_list(self, actors):

        # Update list of obstacles from carla data, should be called before using get_obstacles()
        #
        # Input
        #   
        #   actors:                  list of actors in carla
        #
        #
        # Output
        #   
        #   None
        #
        #

        vehicles_id = []
        for actor in actors:
            if 'vehicle' in actor.type and actor.rolename != 'hero':
                vehicles_id.append(actor.id)   


        topic = "/carla/objects"
        obj_msg = rospy.wait_for_message(topic, ObjectArray, timeout=None)

        self.obstacle_data_list = []

        for obj in obj_msg.objects:
            if obj.id in vehicles_id:
                self.obstacle_data_list.append(obj)

        


    def get_obstacles(self, spline, V_ref, V_threshold_ratio = 0.5):

        # calculate obstacles position and speed, should be called in the planner loop
        #
        # Input
        #   
        #   spline:          	    spline representing the lane
        #   V_ref:                  desired speed of ego vehicle
        #   V_threshold_ratio:      Speed threshold of obstacle vehicle to be considered slow, in percentage to V_ref. Default 50%.
        #   
        #
        # Output
        #   
        #   obs_fast:               List of fast obstacle, each element in form of [(x, y), (s_coordinat, l_coordinate), velocity, velocity_in_l, boundingbox, boundingbox_SL]
        #   obs_slow:               List of slow/oncoming obstacle, each element in form of [(x, y), (s_coordinat, l_coordinate), velocity, velocity_in_l, boundingbox, boundingbox_SL]
        #
        #

           
        obs_slow = []
        obs_fast = []
        
        
        for obstacle in self.obstacle_data_list:
            

            # velocity vector
            v_vector = np.array([obstacle.twist.linear.x, -1 * obstacle.twist.linear.y])

            v = np.sqrt(obstacle.twist.linear.x**2 + obstacle.twist.linear.y**2 + obstacle.twist.linear.z**2)
            


            quaternion = (
                obstacle.pose.orientation.x,
                obstacle.pose.orientation.y,
                obstacle.pose.orientation.z,
                obstacle.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw = -1*euler[2]


            vehicleSize = [obstacle.shape.dimensions[0], obstacle.shape.dimensions[1]]

            boundingbox = get_rect(obstacle.pose.position.x, -1 * obstacle.pose.position.y, yaw, vehicleSize)

            boundingbox_SL = []
            for vertex in boundingbox:
                s, d, tangent, norm = spline.xy2sd(vertex[0,0], vertex[0,1])
                boundingbox_SL.append(np.array([[s,d]]))
                
            
        
            s, d, tangent, norm = spline.xy2sd(obstacle.pose.position.x, -1 * obstacle.pose.position.y)

            # forward or backward along the road
            sign = np.sign( np.dot( v_vector, tangent) )

            # lane changing speed
            vd = np.dot( v_vector, norm) 
            
            # dynamic vehicle
            if v * sign >= V_ref * V_threshold_ratio:

                obs_fast.append( [(obstacle.pose.position.x, -1 * obstacle.pose.position.y), (s, d), v, vd, boundingbox, boundingbox_SL])
                

            # static/slow/oncoming
            if v * sign < V_ref * V_threshold_ratio:

                obs_slow.append( [(obstacle.pose.position.x, -1 * obstacle.pose.position.y), (s, d), v, vd, boundingbox, boundingbox_SL])

        
        self.obstacle = obs_fast+obs_slow
        self.spline = spline

        return obs_fast, obs_slow




    def ST_project_obstacle(self, path_s, path_xy, heading, time, vehicleSize = [5, 3.5], obstacle = None, spline = None):


        

        # project obstacle interaction onto ST space
        #
        # Input
        #
        #   path_s:                 Span of the planned SL path in s coord.; one point sampled per every stepSize in s coord. np.array([s0,s1,...]) 
        #   path_xy:                planned SL path in xy coord., corresponding to each point in s coord. above. np.array([[x0,y0],[x1,y1],...])
        #   heading:                heading/tangent line as unit vector of planned SL path in xy coord., corresponding to each point in s coord. above. np.array([yaw0, yaw1,...])
        #   time:                   timespan of the ST space, one point sampled per every stepSize in t coord. np.array([t0,t1,..])
        #   vehicleSize:            vehicle size in [length, width]
        #   obstacle:               obstacles to be projected, will be automated updated using get_obstacles() without passing any object here. Reserved for testing.
        #                           Each element in form of [(x, y), (s_coordinat, l_coordinate), velocity, velocity_in_l, boundingbox, boundingbox_SL]
        #   spline:          	    spline representing the lane, will be automated updated using get_obstacles() without passing any object here. Reserved for testing.
        #   
        #
        # Output
        #   
        #   obstacle_st:            list of ST coordinate where path overlaps with obstacles. [ [s,t], [s,t], ... ]
        #
        #


        if obstacle == None:
            obstacle = self.obstacle

        if spline == None:
            spline = self.spline
    


        num_of_points = path_xy.shape[0]

        assert path_s.shape[0] == num_of_points
        assert heading.shape[0] == num_of_points

        obstacle_st = []

        s = []
        d = []
        
        for i in range(path_xy.shape[0]):

            pt = path_xy[i,:]

            s, d, tangent, norm = spline.xy2sd(pt[0], pt[1])
           
            
            lane_angle = np.arctan2(tangent[1],tangent[0])
             

            yaw = heading[i] - lane_angle
            

        
            topleft, bottomleft, bottomright, topright = get_rect(s, d, yaw[0], vehicleSize)
            vertices = np.vstack((topleft, bottomleft, bottomright, topright))

            vehicle_poly = tuple(map(tuple, vertices))

            for ob in obstacle:

                velocity =  ob[2]
                velocity_in_l = ob[3]
                boundingbox_SL = ob[5]

                
                for t in time:

                    topleft, bottomleft, bottomright, topright = boundingbox_SL
                    vertices = np.vstack((topleft, bottomleft, bottomright, topright))

                    if abs(velocity_in_l) < 0.333*abs(velocity): # no lane change

                        vertices = vertices + np.array([[velocity, 0]]) * t

                    else:

                        vertices = vertices + np.array([[ np.sqrt(velocity**2 - velocity_in_l**2), velocity_in_l ]]) * t
                    
                    
                    poly = tuple(map(tuple, vertices))

                

                    collide = gjk.collidePolyPoly(vehicle_poly, poly)

                    if collide:

                        obstacle_st.append([path_s[i], t])

        return obstacle_st
