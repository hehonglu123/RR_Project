#!/usr/bin/env python
from RobotRaconteur.Client import *
import time
import numpy as np
import sys
sys.path.append('../../')
from vel_emulate import EmulatedVelocityControl
sys.path.append('../../toolbox')
from sawyer_ik import sawyer_inv
from ur_ik import ur_inv
from abb_ik import abb_inv
from staubli_ik import staubli_inv

R_sawyer=np.array([[ 0., 0., -1. ],
 [ 0., -1.,  0.],
 [-1.,  0., 0.]])
R_UR=np.array([[-1,0,0],
			[0,0,-1],
			[0,-1,0]])
R_abb=np.array([[0,0,1],[0,1,0],[-1,0,0]])
R_staubli=np.array([[ -1, 0., 0 ],
 [ 0., 1,  0.],
 [0,  0., -1]])

robot = RRN.ConnectService('rr+tcp://localhost:58654?service=robot')        #Sawyer
robot3 = RRN.ConnectService('rr+tcp://localhost:58655?service=robot')       #ABB


robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = jog_mode



robot3.command_mode = halt_mode
time.sleep(0.1)
robot3.command_mode = jog_mode




# robot.jog_joint(sawyer_inv(R_sawyer,[0.9,0.0,0.5]).reshape((7,1)), np.ones((7,)), False, False)
robot.jog_joint([0,-0.6,0,0,0,0,0], np.ones((7,)), False, False)


p=abb_inv(R_abb,[0.66,0.5,0.6]).reshape((6,1))
robot3.jog_joint(p, np.ones((6,)), False, False)


