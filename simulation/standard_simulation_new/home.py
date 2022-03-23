#!/usr/bin/env python3
from RobotRaconteur.Client import *
import time
import numpy as np
import sys
sys.path.append('../../')
from vel_emulate import EmulatedVelocityControl
sys.path.append('../../toolbox')
from robots_def import *

Sawyer = RRN.ConnectService('rr+tcp://localhost:58654?service=robot')       #Sawyer
UR = RRN.ConnectService('rr+tcp://localhost:58653?service=robot')       #UR5
ABB = RRN.ConnectService('rr+tcp://localhost:58655?service=robot')       #ABB
Staubli = RRN.ConnectService('rr+tcp://localhost:58656?service=robot')       #Staubli



robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", Sawyer)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
Sawyer.command_mode = halt_mode
time.sleep(0.1)
Sawyer.command_mode = jog_mode

UR.command_mode = halt_mode
time.sleep(0.1)
UR.command_mode = jog_mode

ABB.command_mode = halt_mode
time.sleep(0.1)
ABB.command_mode = jog_mode

Staubli.command_mode = halt_mode
time.sleep(0.1)
Staubli.command_mode = jog_mode

with open('config/abb1200.yml') as file:
	abb_robot=yml2robdef(file)
with open('config/sawyer_robot_default_config.yml') as file:
	sawyer_robot=yml2robdef(file)
with open('config/ur5_robot_default_config.yml') as file:
	ur_robot=yml2robdef(file)
with open('config/staubli_robot_default_config.yml') as file:
	staubli_robot=yml2robdef(file)


from sawyer_ik import inv

Sawyer.jog_freespace(inv([0.4,0.0,0.3]).reshape((7,1)), np.ones((7,)), False)

from ur_ik import inv
p=inv([0.3,0.1,0.1]).reshape((6,1))
UR.jog_freespace(p, np.ones((6,)), False)

from abb_ik import inv

p=inv([0.3,0.0,0.3],R_abb(0)).reshape((6,1))
ABB.jog_freespace(p, np.ones((6,)), False)

from staubli_ik import inv

p=inv([0.3,0.0,0.3],R_staubli(0)).reshape((6,1))
Staubli.jog_freespace(p, np.ones((6,)), False)