#!/usr/bin/python3

from operator import is_not
import sys
import threading
from numpy.core.numeric import count_nonzero
import rospy
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import *
from nav_msgs.msg import *
from math import *
from tf.transformations import euler_from_quaternion
import numpy as np
from scipy.signal import argrelextrema, find_peaks, peak_prominences
import matplotlib.pyplot as plt
import csv

q0 = None
q0_lck = threading.Lock()
qf = None
theta = None
theta_lck = threading.Lock()
lrange = None
lrange_lck = threading.Lock()
range_max = None
angle_min = None
angle_increment = None

d = 0.5

offset = 1.5

d_heuristic_min_prv = inf
qf_prv = None

d_follow = None
d_reach = None
q_Oi_obstacle_prev = None

motion_to_goal_control = True
boundary_following_control = False
met_threshold = False

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def odometry_callback_robot(data):
    global q0, q0_lck, theta, theta_lck, d

    orient = data.pose.pose.orientation
    theta_lck.acquire()
    (roll, pitch, theta) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
    theta_lck.release()
    x0 = data.pose.pose.position.x + d * cos(theta)
    y0 = data.pose.pose.position.y + d * sin(theta)
    q0_lck.acquire()
    q0 = np.array([x0, y0])
    q0_lck.release()

def range_callback_robot(data):
    global lrange, lrange_lck, angle_min, angle_increment, range_max
    lrange_lck.acquire()
    lrange = np.array(data.ranges) - offset
    lrange_lck.release()
    angle_min = data.angle_min
    angle_increment = data.angle_increment
    range_max = data.range_max - offset

def get_continuity_intervals(lrange_cpy):
    max_range_discontinuity = lrange_cpy > range_max - 0.01
    continuity_intervals = []
    first_edge = None
    second_edge = None
    for i in range(0,len(max_range_discontinuity)):
        first = max_range_discontinuity[i]
        second = max_range_discontinuity[(i+1)%len(max_range_discontinuity)]
        if first == True and second == False:
            first_edge = (i+1)%len(max_range_discontinuity)
            #print("First edge: {} Range: {}".format(i+1, lrange_cpy[i+1]))
        if first == False and second == True:
            second_edge = i
            #if first_edge != second_edge:
                #print("Second edge: {} Range: {}".format(i, lrange_cpy[i]))
            continuity_intervals.append(np.array([first_edge, second_edge]))
            first_edge = None
            second_edge = None
    if first_edge is not None and second_edge is None:
        continuity_intervals[0][0] = first_edge
    if len(continuity_intervals) > 0 and continuity_intervals[0][0] is None:
        raise RuntimeError('Failed to get intervals')
    return continuity_intervals

def attraction_potential(qgoal):
    di_threshold = 5
    a = 5
    di = np.linalg.norm(q0-qgoal)
    if di <= di_threshold:
        d_pot = a*(q0-qgoal)
    else:
        d_pot = di_threshold*a*(q0-qgoal)/di    
    return d_pot

def get_interval_boundary_data(qf,continuity_intervals, lrange_cpy, theta_cpy, q0_cpy):
    q_Oi_vec = []
    d_q_Oi_vec = []
    for interval in continuity_intervals:
        for idx in interval:
            d_q_Oi = lrange_cpy[idx]
            q_Oi = q0_cpy + d_q_Oi*np.array([cos(theta_cpy+angle_min+idx*angle_increment), sin(theta_cpy+angle_min+idx*angle_increment)])
            d_q_Oi_vec.append(d_q_Oi)
            q_Oi_vec.append(q_Oi)
    
    qf_is_blocked = False
    intersected_interval = None
    q_Oi_obstacle = None
    for i in range(0,len(q_Oi_vec),2):
        if intersect(q_Oi_vec[i],q_Oi_vec[i+1],q0_cpy,qf):
            qf_is_blocked = True
            q_Oi_obstacle = (q_Oi_vec[i],q_Oi_vec[i+1])
            intersected_interval = continuity_intervals[int(i/2)]
            break
    
    return (qf_is_blocked, q_Oi_vec, d_q_Oi_vec, intersected_interval, q_Oi_obstacle)

def motion_to_goal(qf, alp, k):
    global motion_to_goal_control, boundary_following_control, d_heuristic_min_prv, qf_prv
    lrange_cpy = lrange
    q0_cpy = q0
    theta_cpy = theta
    is_normal = False
    #threshold = 0.1
    #discontinuities_idx = np.where(abs(np.diff(lrange_cpy))>threshold)[0] + 1
    #discontinuities_idx = find_peaks(lrange_cpy, distance=10)
    continuity_intervals = get_continuity_intervals(lrange_cpy)
    #qf_cur = q0_cpy+range_max*(qf-q0_cpy)/dist(qf,q0_cpy)
    qf_cur = qf
    if len(continuity_intervals) > 0:
        data = get_interval_boundary_data(qf,continuity_intervals, lrange_cpy, theta_cpy, q0_cpy)
        qf_is_blocked = data[0]
        q_Oi_vec = data[1]
        d_q_Oi_vec = data[2]
        if qf_is_blocked:
            d_heuristic_min = inf
            for i in range(0,len(q_Oi_vec)):
                q_Oi = q_Oi_vec[i]
                d_q_Oi = d_q_Oi_vec[i]
                d_Oi_goal = dist(qf,q_Oi)
                #print("q_Oi: {},{} d_q_Oi: {} d_Oi_goal: {} heur: {}".format(q_Oi[0],q_Oi[1], d_q_Oi, d_Oi_goal, d_q_Oi+d_Oi_goal))
                if d_Oi_goal < dist(q0_cpy,qf) and d_q_Oi+d_Oi_goal < d_heuristic_min:
                    d_heuristic_min = d_q_Oi+d_Oi_goal
                    qf_cur = q_Oi
            if d_heuristic_min - d_heuristic_min_prv > 0.05:
                d_heuristic_min_prv = inf
                motion_to_goal_control = False
                boundary_following_control = True
                return (None, is_normal)
            else:
                d_heuristic_min_prv = d_heuristic_min

        idx_local_min = np.argmin(lrange_cpy)
        if lrange_cpy[idx_local_min] < 0:
            normal = -np.array([cos(theta_cpy+angle_min+idx_local_min*angle_increment), sin(theta+angle_min+idx_local_min*angle_increment)])
            is_normal = True
            d_heuristic_min_prv = inf
            return (k*normal, is_normal)

    qf_prv = qf_cur
    return (-alp*attraction_potential(qf_cur), is_normal)

def boundary_following(qf, V_prev, k):
    global qf_prv, d_follow, motion_to_goal_control, boundary_following_control, q_Oi_obstacle_prev, met_threshold
    lrange_cpy = lrange
    q0_cpy = q0
    theta_cpy = theta
    is_normal = False

    continuity_intervals = get_continuity_intervals(lrange_cpy)
    data = get_interval_boundary_data(qf,continuity_intervals, lrange_cpy, theta_cpy, q0_cpy)
    obstacle_interval = data[3]
    q_Oi_obstacle = data[4]

    if obstacle_interval is None:
        q_Oi_vec = data[1]
        d_Oi_Oiprev = inf
        for i in range(0,len(q_Oi_vec),2):
            d0 = dist(q_Oi_obstacle_prev[0],q_Oi_vec[i])
            d1 = dist(q_Oi_obstacle_prev[1],q_Oi_vec[i+1])
            if d0+d1 < d_Oi_Oiprev:
                d_Oi_Oiprev = d0+d1
                obstacle_interval = continuity_intervals[int(i/2)]
                q_Oi_obstacle = (q_Oi_vec[i],q_Oi_vec[i+1])
    
    q_Oi_obstacle_prev = q_Oi_obstacle

    if obstacle_interval is not None:
        if obstacle_interval[0] < obstacle_interval[1]:
            rng = range(obstacle_interval[0], obstacle_interval[1])
        else:
            rng = list(range(obstacle_interval[0], len(lrange_cpy)))+list(range(0,obstacle_interval[1]))
    else:
        raise RuntimeError('Failed to detect obstacle interval')

    d_reach = inf
    d_follow_aux = None
    #for idx in range(0,lrange_cpy.shape[0]):
    for idx in rng:
        d_qi = lrange_cpy[idx]
        qi = q0_cpy + d_qi*np.array([cos(theta_cpy+angle_min+idx*angle_increment), sin(theta+angle_min+idx*angle_increment)])
        d_qi_qf = dist(qi,qf)
        if (idx == rng[0] or idx == rng[-1]) and d_qi_qf < d_reach:
            d_reach = d_qi_qf
        if d_qi < range_max and d_follow is not None and d_qi_qf < d_follow:
            d_follow_aux = d_qi_qf
    
    if d_follow is None:
        d_follow = d_reach
    
    #print("d_reach: {} d_follow: {}".format(d_reach,d_follow))
    if d_follow is not inf and d_reach - d_follow < -0.05:
        d_follow = None
        motion_to_goal_control = True
        boundary_following_control = False
        met_threshold = False
        return (None, is_normal)
    else:
        if d_follow_aux is not None:
            d_follow = d_follow_aux
    
    idx_local_min = rng[np.argmin(lrange_cpy[rng])]
    normal = -np.array([cos(theta_cpy+angle_min+idx_local_min*angle_increment), sin(theta+angle_min+idx_local_min*angle_increment)])
    #if (not met_threshold and lrange_cpy[idx_local_min] > 0) or (met_threshold and lrange_cpy[idx_local_min] >= -0.1 and lrange_cpy[idx_local_min] <= 0.1):
    if lrange_cpy[idx_local_min] >= -0.1 and lrange_cpy[idx_local_min] <= 0.1:
        tangent0 = np.array([-normal[1],normal[0]]) # 90º
        phi0 = np.arccos(np.dot(tangent0,V_prev)/(np.linalg.norm(tangent0)+np.linalg.norm(V_prev)))
        tangent1 = np.array([normal[1],-normal[0]]) # -90º
        phi1 = np.arccos(np.dot(tangent1,V_prev)/(np.linalg.norm(tangent1)+np.linalg.norm(V_prev)))
        if phi0 < phi1:
            tangent = tangent0
        else:
            tangent = tangent1
        return (k*tangent, is_normal)
    else:
        met_threshold = True
        is_normal = True
        if lrange_cpy[idx_local_min] < -0.1:
            return (k*normal, is_normal)
        if lrange_cpy[idx_local_min] > 0.1:
            return (-k*normal, is_normal)

def init():
    rospy.init_node('curve_following', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, odometry_callback_robot)
    rospy.Subscriber('/base_scan', LaserScan, range_callback_robot)
    rate = rospy.Rate(20)
    vel_msg = Twist()

    if sys.version_info.major == 2:
        qfx, qfy = raw_input('Digite as coordenadas do alvo (x,y): ').split()
    elif sys.version_info.major == 3:
        qfx, qfy = input('Digite as coordenadas do alvo (x,y): ').split()
    
    qfx, qfy = [float(i) for i in [qfx, qfy]]
    qf = np.array([qfx,qfy])

    q0_hist = []
    V_prev = None
    alp = 0.02
    k = 0.5
    is_normal = False
    e_arrival = 0.01
    e_nosolution = 0.1
    no_solution = False
    arrived = False

    with open('data_tb_test.csv','w', newline='') as csvfile:
        fieldnames = ['time','x','y','theta','Vx','Vy','Vlin','Vang',\
            'isMotionToGoal','isBoundaryFollowing','isNormal']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        while not rospy.is_shutdown() and not no_solution and not arrived:
            if q0 is not None and qf is not None and theta is not None and lrange is not None:
                if motion_to_goal_control:
                    (V, is_normal) = motion_to_goal(qf,alp,k)
                    if not is_normal:
                        print("Motion to goal")
                    else:
                        print("Normal motion")
                if boundary_following_control:
                    (V, is_normal) = boundary_following(qf,V_prev,k)
                    if V is None:
                        continue
                    if not is_normal:
                        print("Boundary following")
                    else:
                        print("Normal motion")

                # Omnidirectional robot
                #vel_msg.linear.x = V[0]
                #vel_msg.linear.y = V[1]

                # Diff robot
                vel_msg.linear.x = cos(theta)*V[0]+sin(theta)*V[1]
                vel_msg.angular.z = (-sin(theta)*V[0]+cos(theta)*V[1])/d
                
                if not is_normal:
                    V_prev = V
                
                writer.writerow({'time':rospy.Time.now(),'x':q0[0],'y':q0[1],\
                    'theta':theta,'Vx':V[0],'Vy':V[1],'Vlin':vel_msg.linear.x,\
                    'Vang':vel_msg.angular.z, 'isMotionToGoal':motion_to_goal_control,\
                    'isBoundaryFollowing': boundary_following_control,'isNormal': is_normal})

                rate.sleep()
                pub.publish(vel_msg)

                no_solution_vec = [np.linalg.norm(q0_h-q0) for q0_h in q0_hist]
                if len(no_solution_vec) > 0:
                    minval = min(no_solution_vec)
                    minpos = np.argmin(no_solution_vec)
                    if minval <= e_nosolution and len(q0_hist)-minpos >= 50:
                        print("No solution")
                        no_solution = True
                if len(q0_hist)==0 or (len(q0_hist) > 0 and not np.array_equal(q0, q0_hist[-1])):
                    q0_hist.append(q0)
                if np.linalg.norm(qf-q0) <= e_arrival:
                    print("Arrived")
                    arrived = True
    print("Finished")

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass