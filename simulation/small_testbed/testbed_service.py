#!/usr/bin/env python3
import time
import numpy as np
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import threading
import traceback
import time, copy


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

		self.model_names=['cube150','cube151','cube152','cube153','cube154','cube155','cube156','cube157']

		self.models=[self.model_names]



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
		self.model_pose["position"]["x"] = obj_pose['position']['x'][0]-self.speed*self.time_diff
		self.model_pose["position"]["y"] = obj_pose['position']['y'][0]
		self.model_pose["position"]["z"] = obj_pose['position']['z'][0]
		model_gz.setf_world_pose(self.model_pose)

		if model_gz.world_pose.PeekInValue()[0]['position'][0]['x']<0:
			self.speed=0
	

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
					if self.reset==1 or not self.model_ready:
						print("Redistribute objects")
						for i in range(len(self.model_names)):
							# reset current four objects pose

							item=self.w.get_models(self.model_names[i])
							theta=0
							self.reset_element(item,np.cos(theta/2.0),np.sin(theta/2.0),i*0.05,0.5,0.1)
						self.model_ready=copy.deepcopy(self.model_names)
						
						self.filled=[0]*self.num_box*self.num_slot
						self.onboard=[]
						self.num_blocks=0
						self.reset=0
					############################slide the box 0.003s per loop
					#drop a new block
					if not self.onboard:
						#add block height
						self.num_blocks+=1
						#new box flag
						self.onboard=[self.model_ready.pop(0)]
						#drop a block
						theta=np.random.random()
						self.reset_element(self.w.get_models(self.onboard[0]),np.cos(theta/2.0),np.sin(theta/2.0),0.5,0.4,0.18)
						self.speed=0.1

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
