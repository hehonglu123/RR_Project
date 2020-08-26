#!/usr/bin/env python3
from RobotRaconteur.Client import *
import time
import numpy as np
import sys
sys.path.append('../../')
from vel_emulate import EmulatedVelocityControl
sys.path.append('../../toolbox')

robot = RRN.ConnectService('rr+tcp://localhost:23333?service=robot')        #rp260



robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = jog_mode


from rp260_ik import inv

p=inv([0.3,0.0,0.2]).reshape((6,1))
robot.jog_joint(p, np.ones((6,)), False, False)