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
        cx, cy, a = raw_input('Digite o centro (x,y) e o parâmetro para curva de Dumbbell (a) ').split()
    elif sys.version_info.major == 3:
        cx, cy, a = input('Digite o centro (x,y) e o parâmetro para curva de Dumbbell (a) ').split()
    
    cx, cy, a = [float(i) for i in [cx, cy, a]]
    t = -1
    w = 0.4

    while not rospy.is_shutdown() and (t+0.005)<1:
        t += 0.005
        xf = a*t+cx
        sqrtval = sqrt(1-pow(t,2))
        yf = a*(pow(t,2))*sqrtval+cy
        Vx = k*(xf-x0)+a
        Vy = k*(yf-y0)+2*a*t*sqrtval-a*pow(t,3)/sqrtval
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

        

    


