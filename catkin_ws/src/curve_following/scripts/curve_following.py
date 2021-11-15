#!/usr/bin/python3

import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import *
from nav_msgs.msg import *
#from turtlesim.msg import Pose
from math import *
from geometry_msgs.msg import PoseStamped

path_pub = rospy.Publisher('/path', Path, queue_size=10)

x0 = 0.0
y0 = 0.0
k = 5
kd = 0.01
d = 0.5
path = Path()

#def pose_callback(data):
#    global x0, y0, theta, k, d
#    x0 = data.x + d * cos(data.theta)
#    y0 = data.y + d * sin(data.theta)
#    theta = data.theta

def odometry_callback(data):
    global x0, y0, theta, k, d, path
    theta = 2*asin(data.pose.pose.orientation.z)
    x0 = data.pose.pose.position.x + d * cos(theta)
    y0 = data.pose.pose.position.y + d * sin(theta)

    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.poses.append(pose)
    path_pub.publish(path)

def init():
    rospy.init_node('curve_following', anonymous=True)
    #pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    #rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rospy.Subscriber('/odom', Odometry, odometry_callback)
    rate = rospy.Rate(20)
    vel_msg = Twist()
    
    if sys.version_info.major == 2:
        cx, cy, a = raw_input('Digite o centro (x,y) e o parâmetro para curva de Cornoid (a) ').split()
    elif sys.version_info.major == 3:
        cx, cy, a = input('Digite o centro (x,y) e o parâmetro para curva de Cornoid (a) ').split()
    
    cx, cy, a = [float(i) for i in [cx, cy, a]]
    t = 0

    while not rospy.is_shutdown():        
        xf = a*cos(t)*(1-2*pow(sin(t),2))+cx
        yf = a*sin(t)*(1+2*pow(cos(t),2))+cy
        Vx = k*(xf-x0)-kd*a*sin(t)*(1-2*pow(sin(t),2)+4*pow(cos(t),2))
        Vy = k*(yf-y0)+kd*a*cos(t)*(1+2*pow(cos(t),2)-4*pow(sin(t),2))
        t += 0.001

        #vel_msg.linear.x = Vx
        #vel_msg.linear.y = Vy
        vel_msg.linear.x = cos(theta)*Vx+sin(theta)*Vy
        vel_msg.angular.z = (-sin(theta)*Vx+cos(theta)*Vy)/d
        rate.sleep()
        pub.publish(vel_msg)
    
if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass

        

    


