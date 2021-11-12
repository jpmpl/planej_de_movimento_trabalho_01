#!/usr/bin/python3

import sys
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import *

x0 = 0.0
y0 = 0.0
k = 0.2
d = 1

def pose_callback(data):
    global x0, y0, theta, k, d
    x0 = data.x + d * cos(data.theta)
    y0 = data.y + d * sin(data.theta)
    theta = data.theta

def init():
    rospy.init_node('curve_following', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rate = rospy.Rate(20)
    vel_msg = Twist()
    
    if sys.version_info.major == 2:
        cx, cy, a = raw_input('Digite o centro (x,y) e o parâmetro para curva de Cornoid (a) ').split()
    elif sys.version_info.major == 3:
        cx, cy, a = input('Digite o centro (x,y) e o parâmetro para curva de Cornoid (a) ').split()
    
    cx, cy, a = [float(i) for i in [cx, cy, a]]
    t = 0
    w = 0.4

    while not rospy.is_shutdown():
        t += 0.05
        xf = a*cos(t)*(1-2*pow(sin(t),2))+cx
        yf = a*sin(t)*(1+2*pow(cos(t),2))+cy
        Vx = k*(xf-x0)-a*sin(t)*(1-2*pow(sin(t),2))-4*a*pow(cos(t),2)*sin(t)
        Vy = k*(yf-y0)+a*cos(t)*(1+2*pow(cos(t),2))-4*a*pow(sin(t),2)*cos(t)
        vel_msg.linear.x = Vx
        vel_msg.linear.y = Vy
        #vel_msg.linear.x = cos(theta)*Vx+sin(theta)*Vy
        #vel_msg.angular.z = (-sin(theta)*Vx+cos(theta)*Vy)/d
        rate.sleep()
        pub.publish(vel_msg)
    
if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass

        

    


