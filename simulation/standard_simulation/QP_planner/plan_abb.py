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
from abb_ik import inv
from general_robotics_toolbox import *      
   
def normalize_dq(q):
    q[:-1]=0.5*q[:-1]/(norm(q[:-1])) 
    return q   

def plan(robot, robot_def ,pd,Rd, vel_ctrl, distance_inst, robot_name,H_robot, obj_vel=[0,0,0], capture_time=0):            #start and end configuration in joint space
    distance_threshold=0.12
    joint_threshold=0.3

    #parameter setup
    n= len(robot.robot_info.joint_info)

    #calc desired joint angles
    q_des=inv(pd,Rd).reshape(n)

    #enable velocity mode
    vel_ctrl.enable_velocity_mode()

    w=10000             #set the weight between orientation and position
    Kq=.01*np.eye(n)    #small value to make sure positive definite
    Kp=np.eye(3)
    KR=np.eye(3)        #gains for position and orientation error
    steps=20           #number of steps to take to get to desired destination

    EP=[1,1,1]
    q_cur=np.zeros(n)

    while(norm(q_des-q_cur)>joint_threshold):
        if norm(obj_vel)!=0:
            p_d=(pd+obj_vel*(time.time()-capture_time))

            q_des=inv(p_d,Rd).reshape(n)
        else:
            p_d=pd
    
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
        EP=p_cur-p_d                             #error in position and orientation

        try:
            distance_report = distance_inst.distance_check(robot_name)
        except:
            traceback.print_exc()
            print("connection to distance checking service lost")

        Closest_Pt=distance_report.Closest_Pt
        Closest_Pt_env=distance_report.Closest_Pt_env
        dist=distance_report.min_distance
        J2C=distance_report.J2C

        if (Closest_Pt[0]!=0. and dist<distance_threshold):  

            print("qp triggering ",dist ) 
            Closest_Pt[:2]=np.dot(H_robot,np.append(Closest_Pt[:2],1))[:2]
            Closest_Pt_env[:2]=np.dot(H_robot,np.append(Closest_Pt_env[:2],1))[:2] 

            k,theta = R2rot(ER)             #decompose ER to (k,theta) pair

        #   set up s for different norm for ER

            s=np.sin(theta/2)*k         #eR2
            vd=-np.dot(Kp,EP)
            wd=-np.dot(KR,s)          
            H=np.dot(np.transpose(Jp),Jp)+Kq #+w*np.dot(np.transpose(JR),JR)
            H=(H+np.transpose(H))/2

            f=-np.dot(np.transpose(Jp),vd)#-w*np.dot(np.transpose(JR),wd)               #setup quadprog parameters


            dx = Closest_Pt_env[0] - Closest_Pt[0]
            dy = Closest_Pt_env[1] - Closest_Pt[1]
            dz = Closest_Pt_env[2] - Closest_Pt[2]

            # derivative of dist w.r.t time
            der = np.array([dx/dist, dy/dist, dz/dist])
            J_Collision=np.hstack((J[3:,:J2C],np.zeros((3,n-J2C))))

            A=np.dot(der.reshape((1,3)),J_Collision)
            
            b=np.array([0.])

            try:
                qdot=normalize_dq(solve_qp(H, f,A,b))
                
            except:
                traceback.print_exc()

        else:
            if norm(q_des-q_cur)<0.5:
                qdot=normalize_dq(q_des-q_cur)
            else:
                qdot=3.*normalize_dq(q_des-q_cur)


        vel_ctrl.set_velocity_command(qdot)

    vel_ctrl.set_velocity_command(np.zeros((n,)))
    vel_ctrl.disable_velocity_mode()  
    return 
