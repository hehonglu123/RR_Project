# -*- coding: utf-8 -*-

import numpy as np
import math
import time
from scipy.linalg import logm, norm, sqrtm
from qpsolvers import solve_qp
from cvxopt import matrix
import traceback
import copy

##RR lib
import RobotRaconteur
from RobotRaconteur.Client import *

import sys
sys.path.append('../toolbox')
from general_robotics_toolbox import *      
   
def normalize_dq(q):
    q[:-1]=0.5*q[:-1]/(norm(q[:-1])) 
    return q   

def plan(robot, robot_def ,vd,Rd, vel_ctrl):            #start and end configuration in joint space
   

    #parameter setup
    n= len(robot.robot_info.joint_info)


    w=10000             #set the weight between orientation and position
    Kq=.01*np.eye(n)    #small value to make sure positive definite
    Kp=np.eye(3)
    KR=np.eye(3)        #gains for position and orientation error
    steps=20           #number of steps to take to get to desired destination

    EP=[1,1,1]
    q_cur=np.zeros(n)

    
    
    #cur joint angle
    q_cur=vel_ctrl.joint_position()

#     get current H and J
    robot_pose=vel_ctrl.robot_pose()
    R_cur = q2R(np.array(robot_pose['orientation'].tolist()))
    p_cur=np.array(robot_pose['position'].tolist())
    J=robotjacobian(robot_def,q_cur)        #calculate current Jacobian
    Jp=J[3:,:]
    JR=J[:3,:]                              #decompose to position and orientation Jacobian

    ER=np.dot(R_cur,np.transpose(Rd))


    k,theta = R2rot(ER)             #decompose ER to (k,theta) pair

#   set up s for different norm for ER

    s=np.sin(theta/2)*k         #eR2
    wd=-np.dot(KR,s)          
    H=np.dot(np.transpose(Jp),Jp)+Kq+w*np.dot(np.transpose(JR),JR)
    H=(H+np.transpose(H))/2

    f=-np.dot(np.transpose(Jp),vd)-w*np.dot(np.transpose(JR),wd)               #setup quadprog parameters


    try:
        qdot=normalize_dq(solve_qp(H, f))
        
    except:
        traceback.print_exc()


    vel_ctrl.set_velocity_command(qdot)

    return 
