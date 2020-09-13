from RobotRaconteur.Client import *
import sys, time, yaml, traceback
import numpy as np
sys.path.append('../')
from vel_emulate_sub import EmulatedVelocityControl


robot_name='sawyer'
url='rr+tcp://bbb1.local:58654?service=sawyer'
robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = position_mode


##robot wire
cmd_w = robot_sub.SubscribeWire("position_command")
state_w = robot_sub.SubscribeWire("robot_state")

vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w, 0.01)
time.sleep(1)
#enable velocity mode
vel_ctrl.enable_velocity_mode()
qdot=np.zeros((7,))
now=time.time()

while True:
	try:
		if time.time()-now<2:
			qdot[0]=-0.5
		elif time.time()-now<4:
			qdot[0]=0.5
		else:
			print('change')
			qdot[0]=0.0	
			now=time.time()	
		vel_ctrl.set_velocity_command(qdot)
		time.sleep(0.01)
	except:
		break
vel_ctrl.set_velocity_command([0,0,0,0,0,0,0])
vel_ctrl.disable_velocity_mode() 
