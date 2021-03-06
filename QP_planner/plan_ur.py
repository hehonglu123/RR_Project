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
from ur_ik import inv
from general_robotics_toolbox import *      
   
def normalize_dq(q):
    q[:-1]=0.5*q[:-1]/(norm(q[:-1])) 
    return q  
def vel_threshold(q):
    super_threshold_indices = q[:-1] > 1.
    super_threshold_indices=np.append(super_threshold_indices,False)
    q[super_threshold_indices] = 1.
    if q[-1]>1.8:
        q[-1]=1.8
    return(q)


def plan(robot, robot_def ,pd,Rd, vel_ctrl, distance_report_wire, robot_name,H_robot, obj_vel=[0,0,0], capture_time=0):            #start and end configuration in joint space
    distance_threshold=0.19
    joint_threshold=0.1

    #parameter setup
    n= len(robot.robot_info.joint_info)

    #calc desired joint angles
    q_des=inv(pd,Rd).reshape(n)

    #enable velocity mode
    vel_ctrl.enable_velocity_mode()

    w=.1                #set the weight between orientation and position
    Kq=.01*np.eye(n)    #small value to make sure positive definite
    Kp=np.eye(3)
    KR=np.eye(3)        #gains for position and orientation error

    EP=[1,1,1]
    q_cur=np.zeros(n)

    while(norm(q_des[:-1]-q_cur[:-1])>joint_threshold):
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
        q_temp=copy.deepcopy(q_cur) 
        q_temp[0]+=np.pi                        #UR configuration
        J=robotjacobian(robot_def,q_temp)        #calculate current Jacobian
        Jp=J[3:,:]
        JR=J[:3,:]                              #decompose to position and orientation Jacobian

        ER=np.dot(R_cur,np.transpose(Rd))
        EP=p_cur-p_d                             #error in position and orientation


        distance_report=distance_report_wire.InValue[robot_name]


        Closest_Pt=distance_report.Closest_Pt
        Closest_Pt_env=distance_report.Closest_Pt_env
        dist=distance_report.min_distance
        J2C=distance_report.J2C

        if (Closest_Pt[0]!=0. and dist<distance_threshold):  
            if J2C>2:
                print("qp triggering ",dist ) 
                Closest_Pt[:2]=np.dot(H_robot,np.append(Closest_Pt[:2],1))[:2]
                Closest_Pt_env[:2]=np.dot(H_robot,np.append(Closest_Pt_env[:2],1))[:2] 

                k,theta = R2rot(ER)             #decompose ER to (k,theta) pair

            #   set up s for different norm for ER

                s=np.sin(theta/2)*k         #eR2
                vd=-np.dot(Kp,EP)
                wd=-np.dot(KR,s)          
                H=np.dot(np.transpose(Jp),Jp)+Kq+w*np.dot(np.transpose(JR),JR)
                H=(H+np.transpose(H))/2

                f=-np.dot(np.transpose(Jp),vd)-w*np.dot(np.transpose(JR),wd)               #setup quadprog parameters


                dx = Closest_Pt_env[0] - Closest_Pt[0]
                dy = Closest_Pt_env[1] - Closest_Pt[1]
                dz = Closest_Pt_env[2] - Closest_Pt[2]

                # derivative of dist w.r.t time
                der = np.array([dx/dist, dy/dist, dz/dist])
                J_Collision=np.hstack((J[3:,:J2C],np.zeros((3,n-J2C))))

                A=np.dot(der.reshape((1,3)),J_Collision)
                
                b=np.array([dist - 0.1])


                try:
                    qdot=1.*normalize_dq(solve_qp(H, f,A,b))
                    if qdot[0]<0:
                        qdot*=1.

                except:
                    traceback.print_exc()
            else:
                qdot=normalize_dq(q_des-q_cur)

        else:
            qdot=normalize_dq(q_des-q_cur)
            if norm(q_des-q_cur)>0.3:
                qdot*=2.5



        vel_ctrl.set_velocity_command(vel_threshold(qdot))

    vel_ctrl.set_velocity_command(np.zeros((n,)))
    vel_ctrl.disable_velocity_mode()  
    return 
