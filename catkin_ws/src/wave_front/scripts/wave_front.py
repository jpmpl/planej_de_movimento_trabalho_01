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
import csv
import cv2 as cv

q0 = None
qf = None
theta = None
lrange = None
k = 10
d = 0.5

class cell:
    def __init__(self, x_img, y_img, x, y, isFree, grid_size, grid_size_r):
        self.isFree = isFree
        self.x_img = x_img
        self.y_img = y_img
        self.x = x
        self.y = y
        self.grid_size = grid_size
        self.grid_size_r = grid_size_r
        self.value = inf
        self.isStart = False
        self.isGoal = False

    def hasPointR(self, q):
        if q[0]>=self.x and q[0]<=self.x+self.grid_size_r and \
                q[1]<=self.y and q[1]>=self.y-self.grid_size_r:
            return True
        else:
            return False

    def getCenterR(self):
        return np.array([self.x+self.grid_size_r/2,self.y+self.grid_size_r/2])
    
def odometry_callback_robot(data):
    global q0, theta, k, d
    orient = data.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
    x0 = data.pose.pose.position.x + d * cos(theta)
    y0 = data.pose.pose.position.y + d * sin(theta)
    q0 = np.array([x0, y0])

def attraction_potential(qgoal):
    di_threshold = 10
    a = 5
    di = np.linalg.norm(q0-qgoal)
    if di <= di_threshold:
        d_pot = a*(q0-qgoal)
    else:
        d_pot = di_threshold*a*(q0-qgoal)/di    
    return d_pot

def init():
    rospy.init_node('curve_following', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, odometry_callback_robot)
    rate = rospy.Rate(20)
    vel_msg = Twist()

    if sys.version_info.major == 2:
        qfx, qfy = raw_input('Digite as coordenadas do alvo (x,y): ').split()
    elif sys.version_info.major == 3:
        qfx, qfy = input('Digite as coordenadas do alvo (x,y): ').split()
    
    qfx, qfy = [float(i) for i in [qfx, qfy]]
    qf = np.array([qfx,qfy])

    img = cv.imread("./catkin_ws/src/wave_front/worlds/circles.png")
    width = img.shape[0]
    height = img.shape[1]
    GRID_SIZE = 50

    ncols = int(width/GRID_SIZE)
    nrows = int(height/GRID_SIZE)
    grid = np.empty([nrows,ncols], cell)
    
    GRID_SIZE_R = 80/ncols

    while q0 is None:
        continue
    rng_x_img = range(0, width, GRID_SIZE)
    rng_y_img = range(0, height, GRID_SIZE)
    rng_x = [x/ncols for x in range(-40*ncols, 40*ncols, int(GRID_SIZE_R*ncols))]
    rng_y = [y/nrows for y in range(40*nrows, -40*nrows, -int(GRID_SIZE_R*nrows))]
    ep_idx = None
    sp_idx = None
    for j in range(0,len(rng_y_img)):
        for i in range(0,len(rng_x_img)):
            cell_rect = img[rng_y_img[j]:rng_y_img[j]+GRID_SIZE,rng_x_img[i]:rng_x_img[i]+GRID_SIZE]
            grid[j,i] = cell(rng_x_img[i], rng_y_img[j],\
                rng_x[i], rng_y[j], not (np.sum(cell_rect==0) > 0),\
                GRID_SIZE, GRID_SIZE_R )
            #print("i:{} j:{}".format(i, j))
            #print("x_img:{} y_img:{}".format(rng_x_img[i], rng_y_img[j]))
            #print("x:{} y:{}".format(rng_x[i], rng_y[j]))
            #cv.imshow('img[{}][{}]'.format(i,j), cell_rect)
            #cv.waitKey(0)
            #cv.destroyAllWindows()
            if grid[j,i].hasPointR(q0):
                if not grid[j,i].isFree:
                    raise RuntimeError('Start is not in a free cell')
                grid[j,i].isStart = True
                sp_idx = (j,i)
            if grid[j,i].hasPointR(qf):
                if not grid[j,i].isFree:
                    raise RuntimeError('Goal is not in a free cell')
                grid[j,i].isGoal = True
                ep_idx = (j,i)

    if ep_idx is None:
        raise RuntimeError('Goal cell not found')
    if sp_idx is None:
        raise RuntimeError('Start cell not found')

    cell_queue = []
    val = 2
    idx = ep_idx
    cur_cell = grid[idx]
    cur_cell.value = val
    cell_queue.append((cur_cell, idx))
    #count = 0
    while True:
        if len(cell_queue) == 0:
            raise RuntimeError('There is no solution')
        cur_cell = cell_queue[0][0]
        idx = cell_queue[0][1]
        cell_queue.pop(0)
        val = cur_cell.value
        if idx[0]+1 < grid.shape[0]:
            if grid[idx[0]+1,idx[1]].isFree and grid[idx[0]+1,idx[1]].value == inf:
                grid[idx[0]+1,idx[1]].value = val+1
                if grid[idx[0]+1,idx[1]].isStart:
                    break
                cell_queue.append((grid[idx[0]+1,idx[1]],(idx[0]+1,idx[1])))
        if idx[1]+1 < grid.shape[1]:
            if grid[idx[0],idx[1]+1].isFree and grid[idx[0],idx[1]+1].value == inf:
                grid[idx[0],idx[1]+1].value = val+1
                if grid[idx[0],idx[1]+1].isStart:
                    break
                cell_queue.append((grid[idx[0],idx[1]+1],(idx[0],idx[1]+1)))
        if idx[0]-1 >= 0:
            if grid[idx[0]-1,idx[1]].isFree and grid[idx[0]-1,idx[1]].value == inf:
                grid[idx[0]-1,idx[1]].value = val+1
                if grid[idx[0]-1,idx[1]].isStart:
                    break
                cell_queue.append((grid[idx[0]-1,idx[1]], (idx[0]-1,idx[1])))
        if idx[1]-1 >= 0:
            if grid[idx[0],idx[1]-1].isFree and grid[idx[0],idx[1]-1].value == inf:
                grid[idx[0],idx[1]-1].value = val+1
                if grid[idx[0],idx[1]-1].isStart:
                    break
                cell_queue.append((grid[idx[0],idx[1]-1], (idx[0],idx[1]-1)))
        #print("ciclo {}".format(count))
        #count+=1

    with open('data_wf_grid_25_13.csv','w', newline='') as csvfile:
        fieldnames = ['row','col','x','y','size','isStart','isGoal','isFree','value']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for j in range(0,grid.shape[0]):
            for i in range(0,grid.shape[1]):
                writer.writerow({'row':j,'col':i,'x':grid[j,i].x,'y':grid[j,i].y,\
                    'size':GRID_SIZE_R,'isStart':grid[j,i].isStart,'isGoal':grid[j,i].isGoal,\
                    'isFree':grid[j,i].isFree,'value': grid[j,i].value})

    ep = 0.005
    alp = 0.5
    
    with open('data_wf_25_13.csv','w', newline='') as csvfile:
        fieldnames = ['time','x','y','theta','Vx','Vy','Vlin','Vang',\
            'grad_atr_pot_x','grad_atr_pot_y','targ_cell_row','targ_cell_col']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        idx = sp_idx
        cur_target = None
        is_in_cur_target_cell = True
        is_in_goal_cell = False
        cur_target_cell = None
        while not rospy.is_shutdown() and np.linalg.norm(q0-qf) > ep:
            if q0 is not None and qf is not None and theta is not None:
                if cur_target is not None and np.linalg.norm(q0-cur_target[0]) < 0.1:
                    idx = cur_target[1]
                    is_in_cur_target_cell = True
                if not is_in_goal_cell and is_in_cur_target_cell:
                    options_4con = []
                    if idx[0]+1 < grid.shape[0]:
                        options_4con.append((grid[idx[0]+1,idx[1]], (idx[0]+1,idx[1])))
                    if idx[1]+1 < grid.shape[1]:
                        options_4con.append((grid[idx[0],idx[1]+1], (idx[0],idx[1]+1)))
                    if idx[0]-1 >= 0:
                        options_4con.append((grid[idx[0]-1,idx[1]], (idx[0]-1,idx[1])))
                    if idx[1]-1 >= 0:
                        options_4con.append((grid[idx[0],idx[1]-1], (idx[0],idx[1]-1)))
                    cur_target_cell = min(options_4con, key= lambda c: c[0].value)
                    cur_target = (cur_target_cell[0].getCenterR(),cur_target_cell[1])
                    is_in_cur_target_cell = False
                    cur_qf = cur_target[0]
                    if cur_target_cell[0].isGoal:
                        is_in_goal_cell = True
                if is_in_goal_cell:
                    cur_qf = qf
                
                d_atr_pot = attraction_potential(cur_qf)
                d_pot = d_atr_pot
                V = - alp*(d_pot)
                    
                # Omnidirectional robot
                #vel_msg.linear.x = V[0]
                #vel_msg.linear.y = V[1]
                
                # Diff robot
                vel_msg.linear.x = cos(theta)*V[0]+sin(theta)*V[1]
                vel_msg.angular.z = (-sin(theta)*V[0]+cos(theta)*V[1])/d

                writer.writerow({'time':rospy.Time.now(),'x':q0[0],'y':q0[1],\
                    'theta':theta,'Vx':V[0],'Vy':V[1],'Vlin':vel_msg.linear.x,\
                    'Vang':vel_msg.angular.z,'grad_atr_pot_x':d_atr_pot[0],\
                    'grad_atr_pot_y':d_atr_pot[1],'targ_cell_row':idx[0],'targ_cell_col':idx[1]})

                rate.sleep()
                pub.publish(vel_msg)
    
if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass