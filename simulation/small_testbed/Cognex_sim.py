#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#Mock-Cognex RR service in Gazebo
import numpy as np
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import sys, traceback, copy, threading, time
sys.path.append('../../toolbox/')
from general_robotics_toolbox import q2R, q2rot, R2rot		#quaternion conversion


class create_impl(object):
	def __init__(self):
		#background threading setup
		self._lock=threading.RLock()
		self._running=False
		#initialize detection object
		self.detection_obj=RRN.NewStructure("edu.rpi.robotics.cognexsim.detection_obj")
		self.H4=np.array([0,0,0,1])
		#connect to gazebo plugin service
		server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
		self.w=server.get_worlds(str(server.world_names[0]))
		#connect to testbed service
		self.testbed_inst=RRN.ConnectService('rr+tcp://localhost:6666?service=testbed') 
		#connect to robot services
		rp260=RRN.ConnectService('rr+tcp://localhost:23333?service=robot')
		self.rp260_state = rp260.robot_state.Connect()

		#initialize detection object dict/map
		self.detection_objects={}
		for model_name in self.testbed_inst.model_names:
			self.detection_objects[model_name]=copy.deepcopy(self.detection_obj)

		self.detection_objects['rp260_eef']=copy.deepcopy(self.detection_obj)

		

	#convert robot eef to world frame
	def convert_to_world(self,robot_state,robot_name):
		#get robot pose
		robot_model=self.w.get_models(robot_name)
		robot_pose=robot_model.world_pose.PeekInValue()[0]
		robot_R=q2R(np.array(robot_pose['orientation'].tolist()[0]))
		robot_P=np.array(robot_pose['position'].tolist()[0]).reshape((3,1))
		robot_H=np.vstack((np.hstack((robot_R,robot_P)),self.H4))


		#get robot ee pose
		ee_R=q2R(np.array(robot_state.InValue.kin_chain_tcp['orientation'].tolist()[0]))
		ee_P=np.array(robot_state.InValue.kin_chain_tcp['position'].tolist()[0]).reshape((3,1))
		ee_H=np.vstack((np.hstack((ee_R,ee_P)),self.H4))

		ee_world_H=np.dot(robot_H,ee_H)
		k,ee_world_angle=R2rot(ee_world_H[:3,:3])

		return ee_world_H[0][-1],ee_world_H[1][-1], ee_world_angle

	def start(self):
		self._running=True
		self._camera = threading.Thread(target=self.update)
		self._camera.daemon = True
		self._camera.start()
	def close(self):
		self._running = False
		self._camera.join()
	def get_pose(self,model_name):
		obj=self.w.get_models(model_name)
		obj_pose=obj.world_pose.PeekInValue()[0]
		x=obj_pose['position']['x'][0]
		y=obj_pose['position']['y'][0]
		quat=np.array(obj_pose['orientation'].tolist()[0])
		k,angle = q2rot(quat)
		return x,y,angle

	def update(self):
		while self._running:
			with self._lock:
				try:
					#iterate all objects to be detected
					for model_name in self.testbed_inst.model_names:
						if (model_name in self.testbed_inst.onboard):
							#update dict of objects 
							x,y,angle=self.get_pose(model_name)
							self.detection_objects[model_name].name=model_name
							self.detection_objects[model_name].detected = True
							self.detection_objects[model_name].x = x
							self.detection_objects[model_name].y = y
							self.detection_objects[model_name].angle = np.degrees(angle)
						else:
							#not detected if it's already on box
							self.detection_objects[model_name].detected = False

					#Read robot eef location 
					
					self.detection_objects['rp260_eef'].x,self.detection_objects['rp260_eef'].y,self.detection_objects['rp260_eef'].angle=self.convert_to_world(self.rp260_state,"rp260")

					#pass to RR wire
					self.time_stamp=time.time()
					self.detection_wire.OutValue=self.detection_objects

				except:
					traceback.print_exc()
	

with RR.ServerNodeSetup("cognexsim_Service", 52222) as node_setup:

	RRN.RegisterServiceTypeFromFile("../../robdef/edu.rpi.robotics.cognexsim")
	create_inst=create_impl()
	create_inst.start()

	RRN.RegisterService("cognexsim", "edu.rpi.robotics.cognexsim.cognexsim", create_inst)
	print("cognex_sim started")
	input("Press enter to quit")
