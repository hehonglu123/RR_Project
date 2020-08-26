#!/usr/bin/env python3

import numpy as np
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s

import traceback

#vacuum 
import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',Attach)
attach_srv.wait_for_service()

detatch_srv = rospy.ServiceProxy('/link_attacher_node/detach',Attach)
detatch_srv.wait_for_service()

class create_impl(object):
	def __init__(self):
		#action list: UR4, ABB4
		self.actions=[0,0,0,0,0,0,0,0]
		self.robot_actions=[0,0,0,0,0,0,0,0]

	def vacuum(self,model_name_1,model_name_2,action):
		if model_name_1=="UR1::urgripper":
			self.actions[0]=action
		elif model_name_1=="UR2::urgripper":
			self.actions[1]=action
		elif model_name_1=="UR3::urgripper":
			self.actions[2]=action
		elif model_name_1=="UR4::urgripper":
			self.actions[3]=action
		elif model_name_1=="ABB1::abbgripper":
			self.actions[4]=action
		elif model_name_1=="ABB2::abbgripper":
			self.actions[5]=action
		elif model_name_1=="ABB3::abbgripper":
			self.actions[6]=action
		elif model_name_1=="ABB4::abbgripper":
			self.actions[7]=action

		req = AttachRequest()
		req.model_name_1=model_name_1
		req.link_name_1 = "body"
		req.model_name_2 = model_name_2
		req.link_name_2 = "link"
		if action:
			attach_srv.call(req)
		else:
			detatch_srv.call(req)

with RR.ServerNodeSetup("vacuum_Service", 50000) as node_setup:

	RRN.RegisterServiceTypeFromFile("../../robdef/edu.rpi.robotics.vacuumlink")

	vacuum=create_impl()
	RRN.RegisterService("vacuumlink", "edu.rpi.robotics.vacuumlink.vacuumlink", vacuum)
	print("vacuum service started......")
	input("Press enter to quit")
