#!/usr/bin/env python
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
		self.detection_obj=RRN.NewStructure("edu.rpi.robotics.cognex.detection_obj")
		self.H4=np.array([0,0,0,1])
		#connect to gazebo plugin service
		server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
		self.w=server.get_worlds(str(server.world_names[0]))
		#connect to testbed service
		self.testbed_inst=RRN.ConnectService('rr+tcp://localhost:6666?service=testbed') 
		#connect to robot services
		ur=RRN.ConnectService('rr+tcp://localhost:58653?service=robot')
		sawyer=RRN.ConnectService('rr+tcp://localhost:58654?service=robot')
		abb=RRN.ConnectService('rr+tcp://localhost:58655?service=robot')
		staubli=RRN.ConnectService('rr+tcp://localhost:58656?service=robot')
		self.ur_state = ur.robot_state.Connect()
		self.sawyer_state = sawyer.robot_state.Connect()
		self.abb_state = abb.robot_state.Connect()
		self.staubli_state = staubli.robot_state.Connect()
		#initialize detection object dict/map
		self.detection_objects={}
		for model_name in self.testbed_inst.model_names:
			self.detection_objects[model_name]=copy.deepcopy(self.detection_obj)
		for i in range(self.testbed_inst.num_box):
			self.detection_objects['box'+str(i)+'t_f']=copy.deepcopy(self.detection_obj)
			self.detection_objects['box'+str(i)+'b_f']=copy.deepcopy(self.detection_obj)
			self.detection_objects['box'+str(i)+'p_f']=copy.deepcopy(self.detection_obj)
			self.detection_objects['box'+str(i)+'s_f']=copy.deepcopy(self.detection_obj)
		self.detection_objects['ur_eef']=copy.deepcopy(self.detection_obj)
		self.detection_objects['sawyer_eef']=copy.deepcopy(self.detection_obj)
		self.detection_objects['abb_eef']=copy.deepcopy(self.detection_obj)
		self.detection_objects['staubli_eef']=copy.deepcopy(self.detection_obj)

		

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
						if not (model_name in self.testbed_inst.onboard):
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
					#iterate all boxes to be detected
					for i in range(self.testbed_inst.num_box):
						
						box_name='box'+str(i)
						x,y,angle=self.get_pose(box_name)

						slot_name=box_name+'t_f'
						if self.testbed_inst.filled[i*self.testbed_inst.num_slot+1]==0:
							self.detection_objects[slot_name].name=slot_name
							self.detection_objects[slot_name].detected = True
							self.detection_objects[slot_name].x = x + 0.042155*np.cos(angle) - 0.067525*np.sin(angle)
							self.detection_objects[slot_name].y = y  + 0.067525*np.cos(angle) + 0.042155*np.sin(angle)
							self.detection_objects[slot_name].angle = np.degrees(angle-np.pi/2)
							self.detection_objects[slot_name].box_idx=i
						else:
							self.detection_objects[slot_name].detected =False

						slot_name=box_name+'p_f'
						if self.testbed_inst.filled[i*self.testbed_inst.num_slot+0]==0:
							self.detection_objects[slot_name].name=slot_name
							self.detection_objects[slot_name].detected = True
							self.detection_objects[slot_name].x = x  + 0.1475*np.cos(angle) - 0.0634*np.sin(angle)
							self.detection_objects[slot_name].y = y  + 0.0634*np.cos(angle) + 0.1475*np.sin(angle)
							self.detection_objects[slot_name].angle = np.degrees(angle-np.pi/2)
							self.detection_objects[slot_name].box_idx=i
						else:
							self.detection_objects[slot_name].detected =False

						slot_name=box_name+'s_f'
						if self.testbed_inst.filled[i*self.testbed_inst.num_slot+2]==0:
							self.detection_objects[slot_name].name=slot_name
							self.detection_objects[slot_name].detected = True
							self.detection_objects[slot_name].x = x  + 0.14922*np.cos(angle) - 0.155628*np.sin(angle)
							self.detection_objects[slot_name].y = y  + 0.155628*np.cos(angle) + 0.14922*np.sin(angle)
							self.detection_objects[slot_name].angle = np.degrees(angle-np.pi/2)
							self.detection_objects[slot_name].box_idx=i
						else:
							self.detection_objects[slot_name].detected =False


						slot_name=box_name+'b_f'

						if self.testbed_inst.filled[i*self.testbed_inst.num_slot+3]==0:
							self.detection_objects[slot_name].name=slot_name
							self.detection_objects[slot_name].detected = True
							self.detection_objects[slot_name].x = x  + 0.042753*np.cos(angle) - 0.164253*np.sin(angle)
							self.detection_objects[slot_name].y = y  + 0.164253*np.cos(angle) + 0.042753*np.sin(angle)
							self.detection_objects[slot_name].angle = np.degrees(angle-np.pi/2)
							self.detection_objects[slot_name].box_idx=i
						else:

							self.detection_objects[slot_name].detected =False

					#Read robot eef location 
					self.detection_objects['ur_eef'].x,self.detection_objects['ur_eef'].y,self.detection_objects['ur_eef'].angle=self.convert_to_world(self.ur_state,"ur")
					self.detection_objects['sawyer_eef'].x,self.detection_objects['sawyer_eef'].y,self.detection_objects['sawyer_eef'].angle=self.convert_to_world(self.sawyer_state,"sawyer")
					self.detection_objects['abb_eef'].x,self.detection_objects['abb_eef'].y,self.detection_objects['abb_eef'].angle=self.convert_to_world(self.abb_state,"abb")
					self.detection_objects['staubli_eef'].x,self.detection_objects['staubli_eef'].y,self.detection_objects['staubli_eef'].angle=self.convert_to_world(self.staubli_state,"staubli")

					#pass to RR wire
					self.time_stamp=time.time()
					self.detection_wire.OutValue=self.detection_objects

				except:
					traceback.print_exc()
	

with RR.ServerNodeSetup("cognex_Service", 52222) as node_setup:

	RRN.RegisterServiceTypeFromFile("../../robdef/edu.rpi.robotics.cognex")
	create_inst=create_impl()
	create_inst.start()

	RRN.RegisterService("cognex", "edu.rpi.robotics.cognex.cognex", create_inst)
	print("cognex_sim started")
	input("Press enter to quit")
