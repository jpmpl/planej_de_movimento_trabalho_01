#!/usr/bin/python3

from operator import is_not
import sys
import rospy
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import *
from nav_msgs.msg import *
from math import *
from tf.transformations import euler_from_quaternion
import numpy as np
from scipy.signal import argrelextrema, find_peaks, peak_prominences
import matplotlib.pyplot as plt

q0 = None
qf = None
theta = None
lrange = None
k = 10
d = 0.5

def odometry_callback_robot(data):
    global q0, theta, k, d
    orient = data.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
    x0 = data.pose.pose.position.x + d * cos(theta)
    y0 = data.pose.pose.position.y + d * sin(theta)
    q0 = np.array([[x0], [y0]])

def range_callback_robot(data):
    global lrange, angle_min, angle_increment
    lrange = np.array(data.ranges)
    angle_min = data.angle_min
    angle_increment = data.angle_increment

""" 
def odometry_callback_goal(data):
    global qf
    xf = data.pose.pose.position.x
    yf = data.pose.pose.position.y
    qf = np.array([[xf], [yf]])
 """

def attraction_potential(qgoal):
    di_threshold = 10
    a = 5
    di = np.linalg.norm(q0-qgoal)
    if di <= di_threshold:
        d_pot = a*(q0-qgoal)
    else:
        d_pot = di_threshold*a*(q0-qgoal)/di    
    return d_pot

def repulsive_potential():
    di_threshold = 7.5
    b = 100
    d_pot = np.array([[0.0], [0.0]])
    #lrange_local_mins = argrelextrema(lrange, np.less, order=20)
    lrange_local_mins = find_peaks(-lrange, height=(-di_threshold, 0), distance=20, prominence=1.0)
    lrange_local_mins = lrange_local_mins[0]
    if lrange_local_mins.size > 0:
        for idx in np.nditer(lrange_local_mins):
            di = np.array([[cos(theta+angle_min+idx*angle_increment-pi)], [sin(theta+angle_min+idx*angle_increment-pi)]])
            d_pot += b*(1/di_threshold - 1/lrange[idx])*1/pow(lrange[idx],2)*di
        return d_pot
    return d_pot

def init():
    rospy.init_node('curve_following', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, odometry_callback_robot)
    rospy.Subscriber('/base_scan', LaserScan, range_callback_robot)
    #rospy.Subscriber('/robot_1/odom', Odometry, odometry_callback_goal)
    rate = rospy.Rate(20)
    vel_msg = Twist()

    if sys.version_info.major == 2:
        qfx, qfy = raw_input('Digite as coordenadas do alvo (x,y): ').split()
    elif sys.version_info.major == 3:
        qfx, qfy = input('Digite as coordenadas do alvo (x,y): ').split()
    
    qfx, qfy = [float(i) for i in [qfx, qfy]]
    qf = np.array([[qfx],[qfy]])
    
    ep = 0.005
    i = 0
    alp = 0.005
    while not rospy.is_shutdown():
        if q0 is not None and qf is not None and theta is not None and lrange is not None:
            if i == 0:
                q = q0
            d_atr_pot = attraction_potential(qf)
            d_rep_pot = repulsive_potential()
            d_pot = d_atr_pot+d_rep_pot
            print("Atractive pot: {}".format(d_atr_pot))
            print("Repulsive pot: {}".format(d_rep_pot))
            if np.linalg.norm(d_pot) > ep:
                V = - alp*(d_pot)
                
                # Omnidirectional robot
                #vel_msg.linear.x = V[0]
                #vel_msg.linear.y = V[1]
                
                # Diff robot
                vel_msg.linear.x = cos(theta)*V[0]+sin(theta)*V[1]
                vel_msg.angular.z = (-sin(theta)*V[0]+cos(theta)*V[1])/d

                i += 1
                rate.sleep()
                pub.publish(vel_msg)
    
if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass