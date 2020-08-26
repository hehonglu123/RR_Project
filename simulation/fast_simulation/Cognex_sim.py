#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#Mock-Cognex RR service in Gazebo
import numpy as np
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import sys, traceback
sys.path.append('../../toolbox/')
from general_robotics_toolbox import q2rot	#convert quaternion to k, theta

host = '128.113.224.144'		#IP address of PC
port = 3000

def get_name(name):
	if "perfume" in name:
		return "pf"
	if "soap" in name:
		return "sp"
	if "toothpaste" in name:
		return "tp"
	if "bottle" in name:
		return "bt"

class create_impl(object):
	def __init__(self):
		self.objects=[]	
		server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
		self.w=server.get_worlds(str(server.world_names[0]))
		self.testbed_inst=RRN.ConnectService('rr+tcp://localhost:6666?service=testbed') 

	def update(self,model_list):
		self.objects=[]								#clean the list
		for i in range(len(model_list)):
			try:
				obj=self.w.get_models(model_list[i])
				obj_pose=obj.world_pose.PeekInValue()[0]
				quat=np.array(obj_pose['orientation'].tolist()[0])
				k,angle = q2rot(quat)

				if "box"==model_list[i][:-1]:
					box_idx=int(model_list[i][-1])-1

					if self.testbed_inst.filled[box_idx*self.testbed_inst.num_slot+1]==0:
						self.objects.append(RR.RobotRaconteurNode.s.NewStructure("edu.rpi.robotics.cognexsim.obj"))
						self.objects[-1].name='t_f'
						self.objects[-1].detect = 1
						self.objects[-1].x = obj_pose['position']['x'][0] + 0.042155*np.cos(angle) - 0.067525*np.sin(angle)
						self.objects[-1].y = obj_pose['position']['y'][0]  + 0.067525*np.cos(angle) + 0.042155*np.sin(angle)
						self.objects[-1].angle = np.degrees(angle-1.57)
						self.objects[-1].obj_pass=1
						self.objects[-1].box_idx=box_idx

					if self.testbed_inst.filled[box_idx*self.testbed_inst.num_slot+0]==0:
						self.objects.append(RR.RobotRaconteurNode.s.NewStructure("edu.rpi.robotics.cognexsim.obj"))
						self.objects[-1].name='p_f'
						self.objects[-1].detect = 1
						self.objects[-1].x = obj_pose['position']['x'][0]  + 0.1475*np.cos(angle) - 0.0634*np.sin(angle)
						self.objects[-1].y = obj_pose['position']['y'][0]  + 0.0634*np.cos(angle) + 0.1475*np.sin(angle)
						self.objects[-1].angle = np.degrees(angle-1.57)
						self.objects[-1].obj_pass=1
						self.objects[-1].box_idx=box_idx

					if self.testbed_inst.filled[box_idx*self.testbed_inst.num_slot+2]==0:
						self.objects.append(RR.RobotRaconteurNode.s.NewStructure("edu.rpi.robotics.cognexsim.obj"))
						self.objects[-1].name='s_f'
						self.objects[-1].detect = 1
						self.objects[-1].x = obj_pose['position']['x'][0]  + 0.14922*np.cos(angle) - 0.155628*np.sin(angle)
						self.objects[-1].y = obj_pose['position']['y'][0]  + 0.155628*np.cos(angle) + 0.14922*np.sin(angle)
						self.objects[-1].angle = np.degrees(angle-1.57)
						self.objects[-1].obj_pass=1
						self.objects[-1].box_idx=box_idx

					if self.testbed_inst.filled[box_idx*self.testbed_inst.num_slot+3]==0:
						self.objects.append(RR.RobotRaconteurNode.s.NewStructure("edu.rpi.robotics.cognexsim.obj"))
						self.objects[-1].name='b_f'
						self.objects[-1].detect = 1
						self.objects[-1].x = obj_pose['position']['x'][0]  + 0.042753*np.cos(angle) - 0.164253*np.sin(angle)
						self.objects[-1].y = obj_pose['position']['y'][0]  + 0.164253*np.cos(angle) + 0.042753*np.sin(angle)
						self.objects[-1].angle = np.degrees(angle-1.57)
						self.objects[-1].obj_pass=1
						self.objects[-1].box_idx=box_idx
					

				else:
					self.objects.append(RR.RobotRaconteurNode.s.NewStructure("edu.rpi.robotics.cognexsim.obj"))
					self.objects[-1].name=model_list[i]
					self.objects[-1].detect = 1
					self.objects[-1].x = obj_pose['position']['x'][0] 
					self.objects[-1].y = obj_pose['position']['y'][0] 
					self.objects[-1].angle = np.degrees(angle)
					self.objects[-1].obj_pass=1

			except:
				print(model_list[i]+" not found")
				traceback.print_exc()



with RR.ServerNodeSetup("cognexsim_Service", 52222) as node_setup:

	RRN.RegisterServiceTypeFromFile("../../robdef/edu.rpi.robotics.cognexsim")
	inst=create_impl()
	# model_list=['perfume_clone_0', 'toothpaste_clone_0', 'soap_clone_0', 'bottle_clone_0','box']

	# inst.update(model_list)
	for i in range(len(inst.objects)):
		print('object name:', inst.objects[i].name)
		print('object detect:', inst.objects[i].detect)
		print('object x:', inst.objects[i].x)
		print('object y:', inst.objects[i].y)
		print('object angle:', inst.objects[i].angle)


	#add authentication for RR connections
	#password: cats111!
	authdata="cats be7af03a538bf30343a501cb1c8237a0 objectlock"
	p=RR.PasswordFileUserAuthenticator(authdata)
	policies={"requirevaliduser" : "true"}
	security=RR.ServiceSecurityPolicy(p,policies)
	RRN.RegisterService("cognexsim", "edu.rpi.robotics.cognexsim.cognexsim", inst,security)
	print("cognex_sim started")
	input("Press enter to quit")
