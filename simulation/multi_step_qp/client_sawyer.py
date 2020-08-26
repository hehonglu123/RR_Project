#pick up and drop detected objects
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np
import time
import traceback
import sys
sys.path.append('../../')
from vel_emulate_sub import EmulatedVelocityControl


with RR.ClientNodeSetup(argv=sys.argv):
	robot_sub = RRN.SubscribeService('rr+tcp://localhost:58654?service=robot')	#ABB

	while True:
		try:
			robot = robot_sub.GetDefaultClient()
			
			break
		except RR.ConnectionException:
			time.sleep(0.1)
	cmd_w=robot_sub.SubscribeWire("position_command")
	state_w=robot_sub.SubscribeWire("robot_state")

	####################Start Service and robot setup
	# robot = RRN.ConnectService('rr+tcp://localhost:58654?service=robot')		#Sawyer
	##########Initialize velocity control parameters
	# cmd_w = robot.position_command.Connect()
	# state_w = robot.robot_state.Connect()
	RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",robot)
	vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w, 0.01)

	robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
	halt_mode = robot_const["RobotCommandMode"]["halt"]
	position_mode = robot_const["RobotCommandMode"]["position_command"]
	jog_mode = robot_const["RobotCommandMode"]["jog"]
	robot.command_mode = halt_mode
	time.sleep(0.1)

	# robot.command_mode = position_mode 
	robot.command_mode = position_mode 
	#enable velocity mode
	vel_ctrl.enable_velocity_mode()
	while True:
		qdot=0.2*np.sin(time.time()*np.ones(7,))
		vel_ctrl.set_velocity_command(qdot)
		# print(vel_ctrl.joint_position())
		time.sleep(0.01)

	vel_ctrl.set_velocity_command(np.zeros((n,)))
	vel_ctrl.disable_velocity_mode()  

