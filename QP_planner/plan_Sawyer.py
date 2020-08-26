# -*- coding: utf-8 -*-

import numpy as np
from scipy.linalg import logm, norm, sqrtm
from ControlParams import *
#import rpi_abb_irc5
from pyquaternion import Quaternion
import quadprog
from cvxopt import matrix
import traceback
import copy
from qp_func import *

##RR lib
import RobotRaconteur
from RobotRaconteur.Client import *

        

def plan(Sawyer,UR, Q_all, vel_ctrl, action):            #start and end configuration in joint space
    distance_inst=RRN.ConnectService('rr+tcp://127.0.0.1:25522?service=Environment')

    #set to velocity control mode
    
    # command_seqno = 1

    R_ee=np.array([[1,0,0],[0,-1,0],[0,0,-1]])      #end effector orientation
    #########Sawyer RR part for velocity command
    
    RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",Sawyer)
    # Initialize Robot Parameters    
    ex,ey,ez,n,P,q_ver,H,ttype,dq_bounds = robotParams("Sawyer")

    # Initialize Control Parameters
    # initial joint angles
    
    
    q = Q_all[0,:].reshape(7, 1)
    R,pos = fwdkin(q,ttype,H,P,n)

    orien = Quaternion(matrix=R)
    orien = np.array([orien[0], orien[1], orien[2], orien[3]]).reshape(1, 4)


    pos_v = np.zeros((3, 1))
    ang_v = np.array([1,0,0,0])
    dq = np.zeros((int(n),1))

    # joint limits
    # Sawyer
    lower_limit = np.transpose(np.array([-90*np.pi/180, -90*np.pi/180, -90*np.pi/180, 225*np.pi/180, -180*np.pi/180, -np.pi,-2*np.pi]))
    upper_limit = np.transpose(np.array([90*np.pi/180, 0*np.pi/180, 90*np.pi/180, 360*np.pi/180, 180*np.pi/180, np.pi,2*np.pi]))
    
    
    # inequality constraints
    h = np.zeros((15, 1))
    sigma = np.zeros((15, 1))
    dhdq = np.vstack((np.hstack((np.eye(7), np.zeros((7, 1)), np.zeros((7, 1)))), np.hstack((-np.eye(7), np.zeros((7, 1)), np.zeros((7, 1)))), np.zeros((1, 9))))

    # velocities
    w_t = np.zeros((3, 1))
    v_t = np.zeros((3, 1))
    
    # keyboard controls
    # define position and angle step
    inc_pos_v = 0.01 # m/s
    inc_ang_v = 0.5*np.pi/180 # rad/s

    # optimization params
    er = 0.05
    ep = 0.05
    epsilon = 0 # legacy param for newton iters
    
    # parameters for inequality constraints
    c = 0.5
    eta = 0.1
    epsilon_in = 0.15
    E = 0.005
    
    Ke = 1
    
    # create a handle of these parameters for interactive modifications
    obj = ControlParams(ex,ey,ez,n,P,H,ttype,dq_bounds,q,dq,pos,orien,pos_v,ang_v.reshape(1, 4),w_t,v_t,epsilon,inc_pos_v,inc_ang_v,0,er,ep,0)
    link_name=0

    pos_b = pos
    R_b = R
    q_b = q
    eu_b = np.array(euler_from_matrix(R_b)) 
    
    Joint_all =[]
    
    ur_angle_prev=0
    for iq in range(Q_all.shape[0]-1):
        vel_ctrl.enable_velocity_mode()
        print("current waypoint: ",iq+1)
        q_a = q_b
        R_a = R_b 
        pos_a = pos_b
        eu_a = eu_b

        
        q_b = Q_all[iq+1,:].reshape(7, 1)
        R_b, pos_b = fwdkin(q_b,ttype,H,P,n)
        eu_b = np.array(euler_from_matrix(R_b))
       
                      
        pp,RR = fwdkin_alljoints(obj.params['controls']['q'],ttype,obj.params['defi']['H'],obj.params['defi']['P'],obj.params['defi']['n'])
        distance_cur=pos_b-np.reshape(pp[:,-1],[3,1])
        distance_q=q_b-obj.params['controls']['q']
        while(norm(distance_cur)>0.04):
            distance_q=q_b-obj.params['controls']['q']
            #update robot joints
            ur_joints=UR.robot_state.PeekInValue()[0].joint_position
            sawyer_joints=vel_ctrl.joint_position() 
            obj.params['controls']['q'] = sawyer_joints.reshape((7,1))
            
            if(ur_joints[0]-ur_angle_prev)>=0:
                ur_action=0
            else:
                ur_action=1

            vel_ctrl.set_velocity_command(obj.params['controls']['dq'].reshape((7,)))
            

            # print(hex(sawyer_state.robot_state_flags))

            Joint_all.append(np.transpose(obj.params['controls']['q'])[0])

            pp,RR = fwdkin_alljoints(obj.params['controls']['q'],ttype,obj.params['defi']['H'],obj.params['defi']['P'],obj.params['defi']['n'])

            distance_cur=pos_b-np.reshape(pp[:,-1],[3,1])

            V_desired=normalize_x(distance_cur)
            
            # parameters for qp
            obj.params['controls']['pos'] = pp[:, -1]

            orien_tmp = Quaternion(matrix=RR[:, :, -1])
            obj.params['controls']['orien'] = np.array([orien_tmp[0], orien_tmp[1], orien_tmp[2], orien_tmp[3]]).reshape(1, 4)

            # ur_joints=np.array([-22.21+180,-88., 134.34,-137.87,-90,151.47])*np.pi/180.           #UR base +/- 180 bias
            #collision checking
            D_ur=ur_joints[0]+0.6
            D_sw=np.pi-sawyer_joints[0]
            try:
                distance_report = distance_inst.distance_check("Sawyer")
            except:
                traceback.print_exc()
                print("connection to distance checking service lost")

            Closest_Pt=distance_report.Closest_Pt
            Closest_Pt_env=distance_report.Closest_Pt_env
            dist=distance_report.min_distance
            link_name=distance_report.robot_link_name

            if (Closest_Pt[0]!=0.):

                # J2C_Joint = Joint2Collision(Closest_Pt, pp)
                J2C_Joint = check_name_sawyer(link_name)
                # print(J2C_Joint)
                J_eef = getJacobian(obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'])
                
                v_tmp = Closest_Pt-obj.params['controls']['pos']
                
                v_tmp2 = (pp[:, -1] - pp[:, -3]) 
                p_norm2 = norm(v_tmp2)
                v_tmp2 = v_tmp2/p_norm2

                J,p_0_tmp = getJacobian2(obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'],Closest_Pt,J2C_Joint)

                # desired rotational velocity
                w_skew = logm(np.dot(RR[:,:,-1], R_b.T))
                w = np.array([w_skew[2, 1], w_skew[0, 2], w_skew[1, 0]])
                vr = -Ke*w
                # vr = -normalize_w(w)
                obj.params['controls']['ang_v'] = vr
                
                # desired linear velocity
                # V_desired = np.reshape(Ke*(x_des-pp[:,-1]),[3,1])
                obj.params['controls']['pos_v'] = V_desired
                
                #print x_des
                #print pp[:,-1]
                #print (x_des-np.reshape(pp[:,-1],[3,1]))
                Q = getqp_H(obj.params['controls']['dq'], J_eef, vr.reshape(3, 1), obj.params['controls']['pos_v'], obj.params['opt']['er'], obj.params['opt']['ep']) 

                # make sure Q is symmetric, PSD
                Q = 0.5*(Q + Q.T) + np.eye(Q[0].size)*1e-5
                # w,v=np.linalg.eig(Q)
                # print(w)

                f = getqp_f(obj.params['controls']['dq'],obj.params['opt']['er'], obj.params['opt']['ep'])
                f = f.reshape((9, ))

                
                # bounds for qp
                if obj.params['opt']['upper_dq_bounds']:
                    bound = obj.params['defi']['dq_bounds'][1, :]
                else:
                    bound = obj.params['defi']['dq_bounds'][0, :]

                LB = np.vstack((-0.1*bound.reshape(7, 1),0,0))
                UB = np.vstack((0.1*bound.reshape(7, 1),1,1))
                LB = matrix(LB, tc = 'd')
                UB = matrix(UB, tc = 'd')
                        
                # inequality constrains A and b
                h[0:7] = obj.params['controls']['q'] - lower_limit.reshape(7, 1)
                h[7:14] = upper_limit.reshape(7, 1) - obj.params['controls']['q']

                dx = Closest_Pt_env[0] - Closest_Pt[0]
                dy = Closest_Pt_env[1] - Closest_Pt[1]
                dz = Closest_Pt_env[2] - Closest_Pt[2]
                
                """ """

                # derivative of dist w.r.t time
                der = np.array([dx/dist, dy/dist, dz/dist])

                """ """
                h[14] = dist - 0.1
                """ """ """ """
                #dhdq[12, 0:6] = np.dot(-der.reshape(1, 3), J_eef2[3:6,:])                

                dhdq[14, 0:7] = np.dot(-der.reshape(1, 3), J[3:6,:])
                
                sigma[0:14] =inequality_bound(h[0:14], c, eta, epsilon_in, E)
                sigma[14] = inequality_bound(h[14], c, eta, epsilon_in, E)           
                
                A = dhdq
                b = sigma

                A = np.vstack((A, np.eye(9), -np.eye(9)))
                b = np.vstack((b, LB, -UB))
                b = b.reshape((33, ))

                # solve the quadprog problem
                #if not is_pos_def(Q):
                #    print np.linalg.eigvals(Q)
                #    dq_sln = np.zeros((6,1))
                #    raw_input('pause')
                #    
                #else:
                try:
                    dq_sln = quadprog.solve_qp(Q, -f, A.T, b)[0]
                except ValueError:
                    obj.params['controls']['dq'] = np.zeros((7,1),dtype=float)
                    obj.params['controls']['dq'][0][0]=normalize_dq_qp(q_b-obj.params['controls']['q'])[0]
                    obj.params['controls']['dq'][1][0]=-0.05
                    obj.params['controls']['dq'][3][0]=0.05
                    obj.params['controls']['dq'][-1][0]=normalize_dq_qp(q_b-obj.params['controls']['q'])[-1]
                    traceback.print_exc()
                    continue
                
                if len(dq_sln) < obj.params['defi']['n']:
                    obj.params['controls']['dq'] = np.zeros((7,1))
                    V_scaled = 0
                    print 'No Solution'
                else:
                    obj.params['controls']['dq'] = dq_sln[0: int(obj.params['defi']['n'])]
                    obj.params['controls']['dq'] = normalize_dq_qp(obj.params['controls']['dq'].reshape((7, 1)))
                    V_scaled = dq_sln[-1]*V_desired
                    vr_scaled = dq_sln[-2]*vr.reshape(3,1)

                
                V_linear = np.dot(J_eef[3:6,:], obj.params['controls']['dq'])
                V_rot = np.dot(J_eef[0:3,:], obj.params['controls']['dq'])
                
                
            else:
                

                obj.params['controls']['dq']=normalize_dq(q_b-obj.params['controls']['q'])
                if (action==0):
                
                    obj.params['controls']['dq']*=0.6*(3*np.pi/4-D_ur)#*np.abs(sawyer_joints[0]+0.8)
                    if (ur_action==1):
                        obj.params['controls']['dq']*=1.5*np.abs(sawyer_joints[0]+1.1)
                else:
                    obj.params['controls']['dq']*=2.


            ur_angle_prev=ur_joints[0]

        vel_ctrl.set_velocity_command(np.zeros((7,)))
        vel_ctrl.disable_velocity_mode()  
    print("moving finished")
    return 


