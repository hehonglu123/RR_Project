#pick up and drop detected objects
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np
import time
import traceback
import copy
import sys
sys.path.append('../../')
from vel_emulate_sub import EmulatedVelocityControl
from jog_joint import jog_joint
sys.path.append('../../toolbox')
from abb_ik import abb_inv as inv
sys.path.append('QP_planner')
from plan_ABB2 import plan

with RR.ClientNodeSetup(argv=sys.argv):
	robot_idx=1
	Rd=np.array([[0,0,1],[0,1,0],[-1,0,0]])


	robot_sub = RRN.SubscribeService('rr+tcp://localhost:58655?service=robot')	#ABB
	distance_inst_sub=RRN.SubscribeService('rr+tcp://localhost:25522?service=Environment')

	while True:
		try:
			robot = robot_sub.GetDefaultClient()
			distance_inst = distance_inst_sub.GetDefaultClient()
			
			break
		except RR.ConnectionException:
			time.sleep(0.1)
	cmd_w=robot_sub.SubscribeWire("position_command")
	state_w=robot_sub.SubscribeWire("robot_state")
	####################Start Service and robot setup
	# robot = RRN.ConnectService('rr+tcp://localhost:58655?service=robot')	 
	# distance_inst=RRN.ConnectService('rr+tcp://localhost:25522?service=Environment')
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

	robot.command_mode = position_mode 
	def single_move(p):
		
		plan(robot,p,Rd, vel_ctrl,distance_inst,robot_idx)
		return

	single_move([0.4,-0.7,0.6])
	time.sleep(0.1)
	single_move([0.66,0.5,0.6])


