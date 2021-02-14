#!/usr/bin/env python

#Frenet optimal trajectory generator

#author: Atsushi Sakai (@Atsushi_twi)

#Ref:

#- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
#(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

#- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
#(https://www.youtube.com/watch?v=Cj6tAQe7UCY)

import time
import rospy
import tf
import numpy as np

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

from std_msgs.msg import Int32

from autoware_msgs.msg import Lane
from autoware_msgs.msg import DetectedObjectArray

import matplotlib.pyplot as plt
import copy
import math
import sys
import os

final_waypoints = Lane()  #this will have the main output 
base_waypoints = Lane()  #this will have the main input
path_wp_x, path_wp_y, path_wp_z= np.array([]), np.array([]), np.array([])
path_wp_vx, path_wp_vy, path_wp_vz= np.array([]), np.array([]), np.array([])
use_init_params = True
current_speed, previous_speed, v, x, y, z, roll, nearest_waypoint_roll, current_course_position= 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
fp = []
obs_coords = np.array([])

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../QuinticPolynomialsPlanner/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../CubicSpline/")

try:
    from quintic_polynomials_planner import QuinticPolynomial
    import cubic_spline_planner
except ImportError:
    raise

SIM_LOOP = 500

# Parameter
MAX_SPEED = 50.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 2.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 1.0  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 7.0  # maximum road width [m]
D_ROAD_W = 1.0  # road width sampling length [m]
DT = 0.2  # time tick [s]
MAX_T = 5.0  # max prediction time [m]
MIN_T = 4.0  # min prediction time [m]
TARGET_SPEED = 30.0 / 3.6  # target speed [m/s]
D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
ROBOT_RADIUS = 2.5  # robot radius [m]

# cost weights
K_J = 0.1
K_T = 0.1
K_D = 1.0
K_LAT = 1.0
K_LON = 1.0

show_animation = True


class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        for Ti in np.arange(MIN_T, MAX_T, DT):
            fp = FrenetPath()

            # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Velocity keeping)
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE,
                                TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = QuarticPolynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2

                tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):
    for fp in fplist:

        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            i_yaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
            fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.hypot(dx, dy))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ob):
    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= ROBOT_RADIUS ** 2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob):
    ok_ind = []
    for i, _ in enumerate(fplist):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            continue
        elif any([abs(a) > MAX_ACCEL for a in
                  fplist[i].s_dd]):  # Max accel check
            continue
        elif any([abs(c) > MAX_CURVATURE for c in
                  fplist[i].c]):  # Max curvature check
            continue
        elif not check_collision(fplist[i], ob):
            continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)
    #fplist = check_paths(fplist, ob)

    # find minimum cost path
    min_cost = float("inf")
    best_path = None
    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp

    return best_path


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp


def waypoints_callback(msg):
	global path_wp_x,path_wp_y,path_wp_z,path_wp_vx,path_wp_vy,path_wp_vz,final_waypoints, nearest_waypoint_roll, base_waypoints
	final_waypoints = msg
	base_waypoints = msg
	l_path_wp_x=np.array([])
	l_path_wp_y=np.array([])
	l_path_wp_z=np.array([])
	l_path_wp_vx=np.array([])
	l_path_wp_vy=np.array([])
	l_path_wp_vz=np.array([])
	nearest_waypoint_quat = (base_waypoints.waypoints[0].pose.pose.orientation.x, base_waypoints.waypoints[0].pose.pose.orientation.y, base_waypoints.waypoints[0].pose.pose.orientation.z, base_waypoints.waypoints[0].pose.pose.orientation.w)
	nearest_waypoint_eul = tf.transformations.euler_from_quaternion(nearest_waypoint_quat)
	nearest_waypoint_roll = nearest_waypoint_eul[0]
	f=open("testing_waypoints.txt", "a+")
	print('No. of BAse Waypoints received: ' + str(len(base_waypoints.waypoints)) + '\n')
	f.write('No. of BAse Waypoints received: ' + str(len(base_waypoints.waypoints)) + '\n')
	for i in range(len(base_waypoints.waypoints)):
		l_path_wp_x = np.append(l_path_wp_x,base_waypoints.waypoints[i].pose.pose.position.x)
		l_path_wp_y = np.append(l_path_wp_y,base_waypoints.waypoints[i].pose.pose.position.y)
		l_path_wp_z = np.append(l_path_wp_z,base_waypoints.waypoints[i].pose.pose.position.z)
		l_path_wp_vx = np.append(l_path_wp_vx,base_waypoints.waypoints[i].twist.twist.linear.x)
		l_path_wp_vy = np.append(l_path_wp_vy,base_waypoints.waypoints[i].twist.twist.linear.y)
		l_path_wp_vz = np.append(l_path_wp_vz,base_waypoints.waypoints[i].twist.twist.linear.z)
	path_wp_x=l_path_wp_x
	path_wp_y=l_path_wp_y
	path_wp_z=l_path_wp_z
	path_wp_vx=l_path_wp_vx
	path_wp_vy=l_path_wp_vy
	path_wp_vz=l_path_wp_vz

def current_pose_callback(msg):	
	global x,y,z,roll
	#received_pose = msg
	x = msg.pose.position.x
	y = (msg.pose.position.y) 
	z = msg.pose.position.z
	quat = ( msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.x)
	eul = tf.transformations.euler_from_quaternion(quat)
	roll = eul[0]
	pitch = eul[1]
	yaw = eul[2]  
	f=open("testing_postion.txt", "a+")
	f.write(str(msg))

def current_velocity_callback(msg):	
	global current_speed, v , previous_speed
	v = msg.twist.linear
	current_speed = np.sqrt(v.x + v.y + v.z)
	#print(len(current_speed))
	f=open("testing_velocity.txt", "a+")
	f.write(str(msg))

def s0_callback(msg):
	global current_course_position
	current_course_position = msg.data

def obstacle_callback(msg):
	global x, y, z, obs_coords #relavent_obstacle_coords_x, relavent_obstacle_coords_y, relavent_obstacle_coords_z
	received_obstacle = msg
	r_o_c_x = []
	r_o_c_y = []
	r_o_c_z = []
	obs_coords_obs = np.array([])
	#f=open("testing_obstacles.txt", "a+")
	#f.write( str(received_obstacle) + '\n')
	#print('No. of Obstacles/objects received: ' + str(len(received_obstacle.objects)) + '\n')
	#f.write('No. of Obstacles/objects received: ' + str(len(received_obstacle.objects)) + '\n')
	for objecti in received_obstacle.objects:
		#if (z-1.0)<=(objecti.pose.position.z)<=(z+1.0):
		if np.sqrt((x-objecti.pose.position.x)**2+(y-objecti.pose.position.y)**2)<20:
			r_o_c_x.append(objecti.pose.position.x)
			r_o_c_y.append(objecti.pose.position.y)
			r_o_c_z.append(objecti.pose.position.z)
	#relavent_obstacle_coords_x = r_o_c_x
	#relavent_obstacle_coords_y = r_o_c_y
	#relavent_obstacle_coords_z = r_o_c_z
	for i in range(len(r_o_c_x)):
		if i==0:
			obs_coords_obs = np.append([obs_coords_obs],[[r_o_c_x[i],r_o_c_y[i]]],axis=1)
		else:
			obs_coords_obs = np.append(obs_coords_obs,[[r_o_c_x[i],r_o_c_y[i]]],axis=0)
	#print(str(obs_coords_obs))
	obs_coords = obs_coords_obs
	#f1=open("testing_full_obstacles.txt", "a+")
	#f1.write(str(received_obstacle))


def main():
    print(__file__ + " start!!")

    # way points
    wx = [0.0, 10.0, 20.5, 35.0, 70.5]
    wy = [0.0, -6.0, 5.0, 6.5, 0.0]
    # obstacle lists
    ob = np.array([[20.0, 10.0],
                   [30.0, 6.0],
                   [30.0, 8.0],
                   [35.0, 8.0],
                   [50.0, 3.0]
                   ])

    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)

    # initial state
    c_speed = 10.0 / 3.6  # current speed [m/s]
    c_d = 2.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    area = 20.0  # animation area length [m]

    for i in range(SIM_LOOP):
        path = frenet_optimal_planning(
            csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)

        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]

        if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(tx, ty)
            plt.plot(ob[:, 0], ob[:, 1], "xk")
            plt.plot(path.x[1:], path.y[1:], "-ob")
            plt.plot(path.x[1], path.y[1], "vr")
            plt.xlim(path.x[1] - area, path.x[1] + area)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
            plt.grid(True)
            plt.pause(0.0001)

    print("Finish")
    if show_animation:  # pragma: no cover
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()

def wp3_frenet():
	global path_wp_x, path_wp_y, previous_speed, c_speed, c_d, c_d_d, c_d_dd, s0, use_init_params
	print("planning frenet path")
	start = time.time()
	fx = []
	fy = []
	
	# way points
	wx = path_wp_x
	wy = path_wp_y
	# obstacle lists
	ob = obs_coords
	
	while not len(wx)==len(wy):
		wx = path_wp_x
		wy = path_wp_y
	
	tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)
	
	if use_init_params:
		# initial state
		c_speed = current_speed/3.6 # current speed [m/s]
		c_d = np.sqrt((x-wx[0])**2 + (y-wy[0])**2)#current lateral position
		c_d_d =  c_speed*np.sin(roll-nearest_waypoint_roll)  #current lateral speed
		c_d_dd = (c_speed - (previous_speed)/3.6)/(start - time.time())**2 # current lateral acceleration [m/s]
		s0 = float(current_course_position)#0.0  # current course position
		use_init_params = False
	
	path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob) #commented obstacles 
	s0 = path.s[1]
	c_d = path.d[1]
	c_d_d = path.d_d[1]
	c_d_dd = path.d_dd[1]
	c_speed = path.s_d[1]
	previous_speed = current_speed
	if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
		print("Goal reached")
	#if len(path.x)>0:
	for i in range(len(path.x)):
		fx.append(path.x[i])
		fy.append(path.y[i])
	#else :
	#	fx = wx
	#	fy = wy
	return fx,fy

if __name__ == '__main__':
	try:
		rospy.init_node('wp3_frenet', anonymous=True)
		
		final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=10)
		
		pose_sub = rospy.Subscriber('/current_pose',PoseStamped,current_pose_callback)
		velo_sub = rospy.Subscriber('/current_velocity',TwistStamped,current_velocity_callback)
		waypoints_sub = rospy.Subscriber('/base_waypoints', Lane,waypoints_callback)
		s0_sub = rospy.Subscriber('/closest_waypoint', Int32,s0_callback)
		obstacles_sub = rospy.Subscriber('/prediction/motion_predictor/objects', DetectedObjectArray,obstacle_callback)
		#main()
		run = 1
		rate = rospy.Rate(10) # 10hz	
		while not rospy.is_shutdown():
			if len(final_waypoints.waypoints)>0:
				final_final_waypoints = final_waypoints
				fx, fy = wp3_frenet()
				time.sleep(0.001)
				#print('No. of fin Waypoints calculated: ' + str(len(fx)) + '\n')
				while not len(final_final_waypoints.waypoints)==len(fx):
					if len(final_final_waypoints.waypoints)<len(fx):
						fx.pop()
						fy.pop()
					if len(final_final_waypoints.waypoints)>len(fx):
						final_final_waypoints.waypoints.pop()
				plt.gcf()
				plt.clf()
				circle1 = plt.Circle((x,y), 50.0, color='c')
				plt.gca().add_patch(circle1)
				print('No. of final Waypoints calculated: ' + str(len(fx)) + '\n')
				
				for i in range(len(fx)):
					final_waypoints.waypoints[i].pose.pose.position.x = fx[i]
					final_waypoints.waypoints[i].pose.pose.position.y = fy[i]
					plt.plot(fx[i],fy[i],"ob")
				final_waypoints_pub.publish(final_waypoints)
				plt.plot(obs_coords[:, 0], obs_coords[:, 1], "xk")
				plt.plot(x,y,"^r")
				plt.plot(path_wp_x,path_wp_y, "og")
				plt.grid(True)
				plt.savefig("{}.jpg".format(run))
				run += 1
				plt.pause(0.001)
			rate.sleep()
		rospy.spin()
	except rospy.ROSInterruptException:
		
		pass