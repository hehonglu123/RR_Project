from RobotRaconteur.Client import *
import sys, time, yaml, traceback
import numpy as np
sys.path.append('../')
from vel_ctrl_sub import VelocityControl
sys.path.append('../toolbox')
from sawyer_ik import inv

def normalize_dq(q):
    q[:-1]=1.*q[:-1]/(np.linalg.norm(q[:-1])) 
    return q   



robot_name='sawyer'
url='rr+tcp://bbb1.local:58654?service=sawyer'
# url='rr+tcp://localhost:58654?service=sawyer'

robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
velocity_mode = robot_const["RobotCommandMode"]["velocity_command"]
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = velocity_mode


##robot wire
cmd_w = robot_sub.SubscribeWire("velocity_command")
state_w = robot_sub.SubscribeWire("robot_state")

vel_ctrl = VelocityControl(robot,state_w, cmd_w)

time.sleep(1)
now=time.time()
change=False

while True:
	try:
		if time.time()-now<5:
			change=False
			q_des=[ 1.42506273, -1.79307395, -0.07933339, 2.53507848, -0.02372143,  0.8282507,-1.7779246 ]

		elif time.time()-now<10:
			q_des=[-0.23359696, -1.69317151, -0.37571268,  2.4358453, -0.06126168,  0.82059891,2.57658593]

		else:
			print('change')
			change=True
			now=time.time()	



		q_cur=vel_ctrl.joint_position()
		
		qdot=normalize_dq(q_des-q_cur)
		if change:
			print(qdot)

		vel_ctrl.set_velocity_command(qdot)

	except:
		traceback.print_exc()
		break
