#!/usr/bin/env python3
import time
import numpy as np
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import threading
import traceback
import time


class create_impl(object):
	def __init__(self):
		self._lock=threading.RLock()
		self._running=False
		self.time=time.time()
		self.time_diff=time.time()-self.time
		server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
		self.w=server.get_worlds(str(server.world_names[0]))
		pose_dtype=RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose", server)
		self.model_pose = np.zeros((1,), dtype = pose_dtype)
		self.reset=1					#reset flag
		self.speed=0.1

		self.num_box=1
		self.num_slot=4
		self.onboard=[]
		self.filled=[0]*self.num_box*self.num_slot

		self.model_names=['perfume','toothpaste','soap','bottle']

		self.models=[self.model_names]

		box2s=[self.w.get_models("box2_1"),self.w.get_models("box2_2")]
		self.box2_pose=[]
		for box2 in box2s:
			self.box2_pose.append(box2.world_pose.PeekInValue()[0]['position'][0])

	def start(self):
		self._running=True
		self._camera = threading.Thread(target=self.testbed)
		self._camera.daemon = True
		self._camera.start()
	def close(self):
		self._running = False
		self._camera.join()
	def model_shift(self,model):
		model_gz=self.w.get_models(model)

		obj_pose=model_gz.world_pose.PeekInValue()[0]

		self.model_pose["orientation"]["w"] = obj_pose["orientation"]["w"][0]
		self.model_pose["orientation"]["z"] = obj_pose["orientation"]["z"][0]
		self.model_pose["position"]["x"] = obj_pose['position']['x'][0]
		self.model_pose["position"]["y"] = obj_pose['position']['y'][0]+self.speed*self.time_diff
		self.model_pose["position"]["z"] = obj_pose['position']['z'][0]
		model_gz.setf_world_pose(self.model_pose)

		if model_gz.world_pose.PeekInValue()[0]['position'][0]['y']>1.8:
			self.reset_model(model,model_gz)
	def reset_model(self,model,model_gz):
		if "box" in model:
			theta=np.pi/8+1.25*np.pi*np.random.random()
			self.reset_element(model_gz,np.cos(theta/2.0),np.sin(theta/2.0),0.5-0.1025*(np.cos(theta)-np.sin(theta)),-2,.9)
			box_idx=int(model[-1])
			self.filled[box_idx*self.num_slot:box_idx*self.num_slot+self.num_slot]=[0]*self.num_slot
			return
		self.onboard.remove(model)
		
		model_series=0

		theta=2*np.pi*np.random.random()
		if "perfume" in model:
			self.reset_element(model_gz,np.cos(theta/2.0),np.sin(theta/2.0),np.clip(np.random.normal(self.box2_pose[2*model_series+1]['x'],0.1),self.box2_pose[2*model_series]['x']-0.05,self.box2_pose[2*model_series+1]['x']+0.05),np.clip(np.random.normal(self.box2_pose[2*model_series+1]['y']+0.05,0.1),self.box2_pose[2*model_series]['y'],self.box2_pose[2*model_series+1]['y']+0.1),self.box2_pose[2*model_series+1]['z']+0.01)
		if "toothpaste" in model:
			self.reset_element(model_gz,np.cos(theta/2.0),np.sin(theta/2.0),np.clip(np.random.normal(self.box2_pose[2*model_series+1]['x'],0.1),self.box2_pose[2*model_series+1]['x']-0.05,self.box2_pose[2*model_series+1]['x']+0.05),np.clip(np.random.normal(self.box2_pose[2*model_series+1]['y']-0.05,0.1),self.box2_pose[2*model_series+1]['y']-0.1,self.box2_pose[2*model_series+1]['y']),self.box2_pose[2*model_series+1]['z']+0.01)
		if "soap" in model:
			self.reset_element(model_gz,np.cos(theta/2.0),np.sin(theta/2.0),np.clip(np.random.normal(self.box2_pose[2*model_series]['x'],0.1),self.box2_pose[2*model_series]['x']-0.05,self.box2_pose[2*model_series]['x']+0.05),np.clip(np.random.normal(self.box2_pose[2*model_series]['y']+0.05,0.1),self.box2_pose[2*model_series]['y'],self.box2_pose[2*model_series]['y']+0.1),self.box2_pose[2*model_series+1]['z']+0.01)
		if "bottle" in model:
			self.reset_element(model_gz,np.cos(theta/2.0),np.sin(theta/2.0),np.clip(np.random.normal(self.box2_pose[2*model_series]['x'],0.1),self.box2_pose[2*model_series]['x']-0.05,self.box2_pose[2*model_series]['x']+0.05),np.clip(np.random.normal(self.box2_pose[2*model_series]['y']-0.05,0.1),self.box2_pose[2*model_series]['y']-0.1,self.box2_pose[2*model_series]['y']),self.box2_pose[2*model_series+1]['z']+0.01)
		return

	def reset_element(self,element,ori_w,ori_z,pos_x,pos_y,pos_z):
		self.model_pose["orientation"]["w"] = ori_w
		self.model_pose["orientation"]["z"] = ori_z
		self.model_pose["position"]["x"] = pos_x
		self.model_pose["position"]["y"] = pos_y
		self.model_pose["position"]["z"] = pos_z
		element.setf_world_pose(self.model_pose)
		return

	def testbed(self):
		while self._running:
			with self._lock:
				try:
					if self.reset==1:
						print("Redistribute objects")
						for i in range(len(self.models)):
							# reset current four objects pose

							model_series=0
							
							#perfume
							perfume=self.w.get_models(self.models[i][0])
							theta=np.random.random()
							self.reset_element(perfume,np.cos(theta/2.0),np.sin(theta/2.0),np.clip(np.random.normal(self.box2_pose[2*model_series+1]['x'],0.1),self.box2_pose[2*model_series]['x']-0.05,self.box2_pose[2*model_series+1]['x']+0.05),np.clip(np.random.normal(self.box2_pose[2*model_series+1]['y']+0.05,0.1),self.box2_pose[2*model_series]['y'],self.box2_pose[2*model_series+1]['y']+0.1),self.box2_pose[2*model_series+1]['z']+0.01)

							#toothpaste
							toothpaste=self.w.get_models(self.models[i][1])
							theta=np.random.random()
							self.reset_element(toothpaste,np.cos(theta/2.0),np.sin(theta/2.0),np.clip(np.random.normal(self.box2_pose[2*model_series+1]['x'],0.1),self.box2_pose[2*model_series+1]['x']-0.05,self.box2_pose[2*model_series+1]['x']+0.05),np.clip(np.random.normal(self.box2_pose[2*model_series+1]['y']-0.05,0.1),self.box2_pose[2*model_series+1]['y']-0.1,self.box2_pose[2*model_series+1]['y']),self.box2_pose[2*model_series+1]['z']+0.01)

							#soap
							soap=self.w.get_models(self.models[i][2])
							theta=np.random.random()
							self.reset_element(soap,np.cos(theta/2.0),np.sin(theta/2.0),np.clip(np.random.normal(self.box2_pose[2*model_series]['x'],0.1),self.box2_pose[2*model_series]['x']-0.05,self.box2_pose[2*model_series]['x']+0.05),np.clip(np.random.normal(self.box2_pose[2*model_series]['y']+0.05,0.1),self.box2_pose[2*model_series]['y'],self.box2_pose[2*model_series]['y']+0.1),self.box2_pose[2*model_series+1]['z']+0.01)

							#bottle
							bottle=self.w.get_models(self.models[i][3])
							theta=np.random.random()
							self.reset_element(bottle,np.cos(theta/2.0),np.sin(theta/2.0),np.clip(np.random.normal(self.box2_pose[2*model_series]['x'],0.1),self.box2_pose[2*model_series]['x']-0.05,self.box2_pose[2*model_series]['x']+0.05),np.clip(np.random.normal(self.box2_pose[2*model_series]['y']-0.05,0.1),self.box2_pose[2*model_series]['y']-0.1,self.box2_pose[2*model_series]['y']),self.box2_pose[2*model_series+1]['z']+0.01)


						# reset box pose
						
						for i in range(self.num_box):
							box_gz=self.w.get_models("box"+str(i))
							self.reset_element(box_gz,np.cos(theta/2.0),np.sin(theta/2.0),0.5-0.1025*(np.cos(theta)-np.sin(theta)),-2,0.9)

						#new box flag
						self.onboard=[]
						self.filled=[0]*self.num_box*self.num_slot

						self.reset=0
					############################slide the box 0.003s per loop
					for i in range(self.num_box):
						self.model_shift("box"+str(i))
					for item in self.onboard:
						self.model_shift(item)

					self.time_diff=time.time()-self.time
					self.time=time.time()
				
				except:
					traceback.print_exc()
with RR.ServerNodeSetup("testbed_Service", 6666) as node_setup:

	RRN.RegisterServiceTypeFromFile("../../robdef/edu.rpi.robotics.testbed")

	service_inst=create_impl()
	service_inst.start()
	RRN.RegisterService("testbed", "edu.rpi.robotics.testbed.testbed", service_inst)
	print("Testbed Service Started")
	input("Press enter to quit")
