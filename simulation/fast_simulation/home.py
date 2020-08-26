#!/usr/bin/env python3
from RobotRaconteur.Client import *
import time
import numpy as np
import sys
import traceback
sys.path.append('../../toolbox')


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

UR1 = RRN.ConnectService('rr+tcp://localhost:55555?service=robot')        
UR2 = RRN.ConnectService('rr+tcp://localhost:55556?service=robot')       
UR3 = RRN.ConnectService('rr+tcp://localhost:55557?service=robot')       
UR4 = RRN.ConnectService('rr+tcp://localhost:55558?service=robot')     
UR=[UR1,UR2,UR3,UR4]
ABB1 = RRN.ConnectService('rr+tcp://localhost:11111?service=robot')        
ABB2 = RRN.ConnectService('rr+tcp://localhost:11112?service=robot')       
ABB3 = RRN.ConnectService('rr+tcp://localhost:11113?service=robot')       
ABB4 = RRN.ConnectService('rr+tcp://localhost:11114?service=robot')   
ABB=[ABB1,ABB2,ABB3,ABB4]

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", UR1)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]

for i in range(4):
	try:
		

		UR[i].command_mode = halt_mode
		time.sleep(0.1)
		UR[i].command_mode = jog_mode
		ABB[i].command_mode = halt_mode
		time.sleep(0.1)
		ABB[i].command_mode = jog_mode
		from ur_ik import inv
		p=inv([0.0,0.3,0.1]).reshape((6,1))
		UR[i].jog_joint(p, np.ones((6,)), False, False)
		from abb_ik import inv
		p=inv([0.3,0.0,0.3]).reshape((6,1))
		ABB[i].jog_joint(p, np.ones((6,)), False, False)
	except:
		traceback.print_exc()
