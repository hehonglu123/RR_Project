#Simple example Robot Raconteur Sawyer Robot and Cognex client
#pick up and drop detected objects
from RobotRaconteur.Client import *
import numpy as np
import time
import traceback
import copy
import sys
from vel_emulate import EmulatedVelocityControl
sys.path.insert(1,"toolbox")
from sawyer_ik import fwd, ik
sys.path.insert(1,"QP_planner")
from plan_Sawyer import plan

Rd=np.array([[ 0., 0., -1. ],
 [ 0., -1.,  0.],
 [-1.,  0., 0.]])

####################Start Service and robot setup
robot = RRN.ConnectService('rr+tcp://127.0.0.1:58653?service=sawyer')


robot.enable()
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
robot.command_mode = halt_mode
time.sleep(0.1)

###velocity control parameters
cmd_w = robot.position_command.Connect()
state_w = robot.robot_state.Connect()
RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",robot)
vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w, 0.01)


robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode=jog_mode
robot.jog_joint(ik(Rd,[-0.2,0.4,0.25]).reshape((7,1)), np.ones((7,)), False, True)

