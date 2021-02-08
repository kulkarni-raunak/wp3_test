#!/usr/bin/env python

#Frenet optimal trajectory generator

#author: Atsushi Sakai (@Atsushi_twi)

#Ref:

#- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
#(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

#- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
#(https://www.youtube.com/watch?v=Cj6tAQe7UCY)


import rospy
import tf
import numpy as np

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

from autoware_msgs.msg import Lane
from autoware_msgs.msg import DetectedObjectArray

import matplotlib.pyplot as plt
import copy
import math
import sys
import os

final_waypoints = Lane()  #this is the main output to the control to follow

path_wp_x, path_wp_y, path_wp_z= np.array([]), np.array([]), np.array([])
path_wp_vx, path_wp_vy, path_wp_vz= np.array([]), np.array([]), np.array([])

current_speed, v= 0.0 , 0.0


#sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                "/../QuinticPolynomialsPlanner/")
#sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                "/../CubicSpline/")

# clone github repository URL = 'https://github.com/akhilkanduri/Github_WP3_Autoware.git' : Add this to Docker File

URL = 'https://github.com/akhilkanduri/Github_WP3_Autoware.git'
# import git # to clone if not added in dockerfile
# git.Git.clone(URL)

# method 1

CurrentDirectory= os.getcwd()
print(os.getcwd())
Directory_List = [x[0] for x in os.walk(os.getcwd())]
print(Directory_List)  # Just for reference to check directory

try:
    from quintic_polynomials_planner import QuinticPolynomial
    import cubic_spline_planner
except ImportError:
   raise
   print('Error: Check Path')
    

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
ROBOT_RADIUS = 2.0  # robot radius [m]

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
	global path_wp_x,path_wp_y,path_wp_z,path_wp_vx,path_wp_vy,path_wp_vz,final_waypoints
	final_waypoints = msg
	received_waypoints = msg
	f=open("testing_waypoints.txt", "a+")
	f.write('No. of Waypoints received: ' + str(len(received_waypoints.waypoints)) + '\n')
	for i in range(len(received_waypoints.waypoints)):
		f.write(str(received_waypoints.waypoints[i]) + '\n')
		path_wp_x = np.append(path_wp_x,received_waypoints.waypoints[i].pose.pose.position.x)
		path_wp_y = np.append(path_wp_y,received_waypoints.waypoints[i].pose.pose.position.y)
		path_wp_z = np.append(path_wp_z,received_waypoints.waypoints[i].pose.pose.position.z)
		path_wp_vx = np.append(path_wp_vx,received_waypoints.waypoints[i].twist.twist.linear.x)
		path_wp_vy = np.append(path_wp_vy,received_waypoints.waypoints[i].twist.twist.linear.y)
		path_wp_vz = np.append(path_wp_vz,received_waypoints.waypoints[i].twist.twist.linear.z)
	print('waypoints received') 

def current_pose_callback(msg):	
	global x,y,z	
	#received_pose = msg
	x = msg.pose.position.x
	y = (msg.pose.position.y) #not sure to multiply by -1 or not
	z = msg.pose.position.z
	quat = ( msg.pose.orientation.x, msg.pose.orientation.x, msg.pose.orientation.x, msg.pose.orientation.x)
	eul = tf.transformations.euler_from_quaternion(quat)
	roll = eul[0]
	pitch = eul[1]
	yaw = eul[2]  #not sure to multiply by -1 or not
	f=open("testing_postion.txt", "a+")
	f.write(str(msg))

def current_velocity_callback(msg):	
	global current_speed, v
	v = msg.twist.linear
	current_speed = np.sqrt(v.x + v.y + v.z)
	#print(len(current_speed))
	f=open("testing_velocity.txt", "a+")
	f.write(str(msg))

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
	print(__file__ + " start!!")

	# way points
	wx = path_wp_x
	wy = path_wp_y
	# obstacle lists
	ob = np.array([])

	tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)

	# initial state
	c_speed = current_speed # current speed [m/s]
	c_d = np.sqrt((x-path_wp_x[0])**2 + (y-path_wp_y[0])**2)#current lateral position
	c_d_d =  np.sqrt((v.x-path_wp_vx[0])**2 + (v.y-path_wp_vy[0])**2) #current lateral speed
	c_d_dd = 0.0  # current lateral acceleration [m/s]
	s0 = 0.0  # current course position
	path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob) #commented obstacles 
	
	if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
		print("Goal reached")
	for i in range(len(final_waypoints.waypoints)):
		final_waypoints.waypoints[i].pose.pose.position.x = path.x[i]
		final_waypoints.waypoints[i].pose.pose.position.y = path.y[i]

if __name__ == '__main__':
	try:
		rospy.init_node('wp3_frenet', anonymous=True)
		
		final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=10)
		
		pose_sub = rospy.Subscriber('/current_pose',PoseStamped,current_pose_callback)
		velo_sub = rospy.Subscriber('/current_velocity',TwistStamped,current_velocity_callback)
		waypoints_sub = rospy.Subscriber('/base_waypoints', Lane,waypoints_callback)
		
		#main()
		
		rate = rospy.Rate(10) # 10hz	
		while not rospy.is_shutdown():
			if final_waypoints.waypoints:
				wp3_frenet()
				fp = final_waypoints
				final_waypoints_pub.publish(fp)
			rate.sleep()
		rospy.spin()
	except rospy.ROSInterruptException:
		
		pass