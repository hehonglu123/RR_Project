#!/usr/bin/env python3
from RobotRaconteur.Client import *
import time
import numpy as np
import sys
sys.path.append('../../')
from vel_emulate import EmulatedVelocityControl
sys.path.append('../../toolbox')

# robot = RRN.ConnectService('rr+tcp://localhost:58654?service=robot')        #Sawyer
robot2 = RRN.ConnectService('rr+tcp://localhost:58653?service=robot')       #UR5
# robot3 = RRN.ConnectService('rr+tcp://localhost:58655?service=robot')       #ABB
# robot4 = RRN.ConnectService('rr+tcp://localhost:58656?service=robot')       #Staubli


robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot2)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
# robot.command_mode = halt_mode
# time.sleep(0.1)
# robot.command_mode = jog_mode

robot2.command_mode = halt_mode
time.sleep(0.1)
robot2.command_mode = jog_mode

# robot3.command_mode = halt_mode
# time.sleep(0.1)
# robot3.command_mode = jog_mode

# robot4.command_mode = halt_mode
# time.sleep(0.1)
# robot4.command_mode = jog_mode

# from sawyer_ik import inv

# robot.jog_joint(inv([0.1,0.3,0.2]).reshape((7,1)), np.ones((7,)), False, False)

from ur_ik import inv
p=inv([0.3,0.1,0.1]).reshape((6,1))
robot2.jog_joint(p, np.ones((6,)), False, False)
# from abb_ik import inv

# p=inv([0.3,0.0,0.3]).reshape((6,1))
# robot3.jog_joint(p, np.ones((6,)), False, False)

# from staubli_ik import inv

# p=inv([0.3,0.0,0.3]).reshape((6,1))
# robot4.jog_joint(p, np.ones((6,)), False, False)