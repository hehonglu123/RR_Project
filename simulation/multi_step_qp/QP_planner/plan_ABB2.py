# -*- coding: utf-8 -*-

import numpy as np
import math
import time
from scipy.linalg import logm, norm, sqrtm
from qpsolvers import solve_qp
from cvxopt import matrix
import traceback
import copy
from qp_func import *

##RR lib
import RobotRaconteur
from RobotRaconteur.Client import *

import sys
sys.path.append('../toolbox')
from abb_ik import abb_inv as inv
from abb_ik import abb_fwd as fwd
from general_robotics_toolbox import *  
sys.path.append('../../')
from jog_joint import jog_joint    
   

def plan(robot,pd,Rd, vel_ctrl, distance_inst, robot_idx, obj_vel=[0,0,0], capture_time=0):            #start and end configuration in joint space
    
    distance_threshold=0.1
    joint_threshold=0.3
    transformations=distance_inst.transformations
    H_robot=transformations[robot_idx].H.reshape((transformations[robot_idx].row,transformations[robot_idx].column))

    #parameter setup
    n= len(robot.robot_info.joint_info)
    P=np.transpose(np.array(robot.robot_info.chains[0].P.tolist()))
    H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
    joint_type = np.zeros(n)
    robot_def=Robot(H,P,joint_type)

    #calc desired joint angles
    q_des=inv(Rd,pd).reshape(n)

    #enable velocity mode
    # vel_ctrl.enable_velocity_mode()
    time_step=0.05
    steps=20
    w=100             #set the weight between orientation and position
    Kq=.01*np.eye(steps*n)    #small value to make sure positive definite
    alpha=0.1
    KR=np.eye(3)        #gains for position and orientation error
    Kp=0.1*np.eye(3)

    EP=[1,1,1]
    q_cur=robot.robot_state.PeekInValue()[0].joint_position
    robot_pose=robot.robot_state.PeekInValue()[0].kin_chain_tcp[0]
    p_cur=np.array(robot_pose['position'].tolist())

    p_d=pd

    
    cur_step=0
    ####initial guess
    qdot=np.tile(normalize_dq_qp(q_des-q_cur),(steps,1))
    q_pred=[]
    p_pred=[]
    q_pred.append(copy.deepcopy(q_cur))
    Transform=fwd(q_cur)
    p_pred.append(Transform.p)
    for i in range(steps-1):
        q_pred.append(q_pred[-1]+qdot[i])
        Transform=fwd(q_pred[-1])
        p_pred.append(Transform.p)  

    while(norm(np.array(p_cur)-np.array(p_d))>0.05):

        step_time=time.time()
        #cur joint angle
        q_cur=robot.robot_state.PeekInValue()[0].joint_position

        robot_pose=robot.robot_state.PeekInValue()[0].kin_chain_tcp[0]
        p_cur=np.array(robot_pose['position'].tolist())
        R_cur = q2R(np.array(robot_pose['orientation'].tolist()))
        

        #multi-step planning:

        
        H=np.zeros((3,n*steps))
        Jp_all_qp=np.zeros((3*steps,n*steps))
        JR_all_qp=np.zeros((3*steps,n*steps))
        
        A=np.zeros((steps,steps*n))
        b=np.zeros((steps,))

        vd=np.zeros((steps,3))
        wd=np.zeros((steps,3))
        #prediction timestamp
        now=time.time()
        #propagate forward:
        for i in range(steps):
            #form EP and ER
            Transform=fwd(q_pred[i])
            EP=Transform.p-p_d
            vd[i]=-np.dot(Kp,EP)                             #error in position and orientation
            ER=np.dot(Transform.R,np.transpose(Rd))
            k,theta = R2rot(ER)             #decompose ER to (k,theta) pair
            try:
                s=np.sin(theta/2)*k         #eR2
            except:
                s=np.sin(theta/2)*np.array(k)         #eR2


            wd[i]=-np.dot(KR,s)

            #     get current H and J
            

            J=robotjacobian(robot_def,q_pred[i])        #calculate current Jacobian
            Jp=J[3:,:]
            JR=J[:3,:]                              #decompose to position and orientation Jacobian
            Jp_all_qp[3*i:3*(i+1),n*i:n*(i+1)]=Jp
            JR_all_qp[3*i:3*(i+1),n*i:n*(i+1)]=JR

            try:
                distance_report = distance_inst.distance_check(robot_idx,q_pred[i],0.2*(i+1))
            except:
                traceback.print_exc()
                print("connection to distance checking service lost")

            Closest_Pt=distance_report.Closest_Pt
            Closest_Pt_env=distance_report.Closest_Pt_env
            dist=distance_report.min_distance
            J2C=distance_report.J2C

            if (Closest_Pt[0]!=0. and dist<distance_threshold):  
                # dist=np.abs(dist)
                Closest_Pt[:2]=np.dot(H_robot,np.append(Closest_Pt[:2],1))[:2]
                Closest_Pt_env[:2]=np.dot(H_robot,np.append(Closest_Pt_env[:2],1))[:2] 

                

                dx = Closest_Pt_env[0] - Closest_Pt[0]
                dy = Closest_Pt_env[1] - Closest_Pt[1]
                dz = Closest_Pt_env[2] - Closest_Pt[2]



                # derivative of dist w.r.t time
                der = np.array([dx/dist, dy/dist, dz/dist])
                J_Collision=np.hstack((J[3:,:J2C],np.zeros((3,n-J2C))))
                A[i,6*i:6*(i+1)]=np.dot(der.reshape((1,3)),J_Collision)
                b[i]=-0.001/dist
                if dist<0:
                    b[i]=-0.2

                
        #form qp parameters
        # vd=-np.dot(Kp,EP)
        
        f=-np.dot(np.transpose(Jp_all_qp),vd.flatten())-w*np.dot(np.transpose(JR_all_qp),wd.flatten())               #setup quadprog parameters

        H=np.dot(np.transpose(Jp_all_qp),Jp_all_qp)+Kq+w*np.dot(np.transpose(JR_all_qp),JR_all_qp)
        H=(H+np.transpose(H))/2
        try:
            qdot=solve_qp(H, f,A,b).reshape((steps,n))#,None,None,lb=-0.5*np.ones(n*steps),ub=0.5*np.ones(n*steps)).reshape((steps,n))
            
        except:
            traceback.print_exc()
            if(dist<0):
                print("here")

        #gradient descent
        q_pred_act=np.array(q_pred+qdot)

        for i in range(steps):
            qdot[i]=normalize_dq(q_pred_act[i]-q_pred[i])/5.
            q_pred_act[i]=q_pred[i]+qdot[i]

        #planning end

                
        # vel_ctrl.set_velocity_command(qdot[0])
        
        # robot.jog_joint(q_pred_act[cur_step], np.ones((n,)), False, True)
        jog_joint(robot,vel_ctrl,q_pred_act[0],time_step)

        # cur_step+=1
        # if cur_step==steps:
        #     cur_step=0

        #move 1 more step in horizon
        q_pred=copy.deepcopy(q_pred_act)

        #same rate
        time.sleep(0.2-(time.time()-step_time))
        
    vel_ctrl.set_velocity_command(np.zeros((n,)))
    vel_ctrl.disable_velocity_mode()  
    return 
