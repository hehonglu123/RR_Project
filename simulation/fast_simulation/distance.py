#!/usr/bin/env python3
import tesseract
import os
import re
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import yaml
import numpy as np
import time
import traceback
import thread
import threading
import sys
sys.path.append('../')
from gazebo_model_resource_locator import GazeboModelResourceLocator

#convert 4x4 H matrix to 3x3 H matrix and inverse for mapping obj to robot frame
def H42H3(H):
	H3=np.linalg.inv(H[:2,:2])
	H3=np.hstack((H3,-np.dot(H3,np.array([[H[0][-1]],[H[1][-1]]]))))
	H3=np.vstack((H3,np.array([0,0,1])))
	return H3

class create_impl(object):
	def __init__(self):
		self._lock=threading.RLock()
		self._running=False
		self.num_robot=8
		#load calibration parameters
		with open("calibration/UR1.yaml") as file:
			H_UR1 		= np.array(yaml.load(file)["H"],dtype=np.float64)
		with open("calibration/UR2.yaml") as file:
			H_UR2 		= np.array(yaml.load(file)["H"],dtype=np.float64)
		with open("calibration/UR3.yaml") as file:
			H_UR3 		= np.array(yaml.load(file)["H"],dtype=np.float64)
		with open("calibration/UR4.yaml") as file:
			H_UR4 		= np.array(yaml.load(file)["H"],dtype=np.float64)

		with open("calibration/ABB1.yaml") as file:
			H_ABB1 	= np.array(yaml.load(file)["H"],dtype=np.float64)
		with open("calibration/ABB2.yaml") as file:
			H_ABB2 	= np.array(yaml.load(file)["H"],dtype=np.float64)
		with open("calibration/ABB3.yaml") as file:
			H_ABB3 	= np.array(yaml.load(file)["H"],dtype=np.float64)
		with open("calibration/ABB4.yaml") as file:
			H_ABB4 	= np.array(yaml.load(file)["H"],dtype=np.float64)


		H_list=[H42H3(H_UR1),H42H3(H_UR2),H42H3(H_UR3),H42H3(H_UR4),H42H3(H_ABB1),H42H3(H_ABB2),H42H3(H_ABB3),H42H3(H_ABB4)]


		
		transformation=RRN.GetStructureType("edu.rpi.robotics.distance.transformation")
		self.transformations=[]

		self.robot_link_list=[]
		self.robot_joint_list=[]
		for i in range(self.num_robot):
			#form H into RR transformation struct
			
			transformation1=transformation()
			transformation1.name="robot"+str(i)
			transformation1.row=len(H_list[i])
			transformation1.column=len(H_list[i][0])
			transformation1.H=np.float16(H_list[i]).flatten().tolist()
			
			self.transformations.append(transformation1)

			if i<4:
				UR_joint_names=["UR"+str(i+1)+"_shoulder_pan_joint","UR"+str(i+1)+"_shoulder_lift_joint","UR"+str(i+1)+"_elbow_joint","UR"+str(i+1)+"_wrist_1_joint","UR"+str(i+1)+"_wrist_2_joint","UR"+str(i+1)+"_wrist_3_joint"]
				UR_link_names=["UR"+str(i+1)+"_base_link","UR"+str(i+1)+"_shoulder_link","UR"+str(i+1)+"_upper_arm_link","UR"+str(i+1)+"_forearm_link","UR"+str(i+1)+"_wrist_1_link","UR"+str(i+1)+"_wrist_2_link","UR"+str(i+1)+"_wrist_3_link"]
				self.robot_link_list.append(UR_link_names)
				self.robot_joint_list.append(UR_joint_names)
			else:
				ABB_joint_names=["ABB"+str(i-3)+"_joint_1","ABB"+str(i-3)+"_joint_2","ABB"+str(i-3)+"_joint_3","ABB"+str(i-3)+"_joint_4","ABB"+str(i-3)+"_joint_5","ABB"+str(i-3)+"_joint_6"]
				ABB_link_names=["ABB"+str(i-3)+"_base_link","ABB"+str(i-3)+"_link_1","ABB"+str(i-3)+"_link_2","ABB"+str(i-3)+"_link_3","ABB"+str(i-3)+"_link_4","ABB"+str(i-3)+"_link_5","ABB"+str(i-3)+"_link_6"]
				self.robot_link_list.append(ABB_link_names)
				self.robot_joint_list.append(ABB_joint_names)



		#Connect to robot service
		UR1 = RRN.ConnectService("rr+tcp://localhost:55555?service=robot")        
		UR2 = RRN.ConnectService("rr+tcp://localhost:55556?service=robot")       
		UR3 = RRN.ConnectService("rr+tcp://localhost:55557?service=robot")       
		UR4 = RRN.ConnectService("rr+tcp://localhost:55558?service=robot")   

		ABB1 = RRN.ConnectService("rr+tcp://localhost:11111?service=robot")        
		ABB2 = RRN.ConnectService("rr+tcp://localhost:11112?service=robot")       
		ABB3 = RRN.ConnectService("rr+tcp://localhost:11113?service=robot")       
		ABB4 = RRN.ConnectService("rr+tcp://localhost:11114?service=robot") 


		UR1_state=UR1.robot_state.Connect()
		UR2_state=UR2.robot_state.Connect()
		UR3_state=UR3.robot_state.Connect()
		UR4_state=UR4.robot_state.Connect()

		ABB1_state=ABB1.robot_state.Connect()
		ABB2_state=ABB2.robot_state.Connect()
		ABB3_state=ABB3.robot_state.Connect()
		ABB4_state=ABB4.robot_state.Connect()


		self.robot_state_list=[UR1_state,UR2_state,UR3_state,UR4_state,ABB1_state,ABB2_state,ABB3_state,ABB4_state]
		self.L2C=[''] * self.num_robot
		self.distance_matrix=-np.ones(self.num_robot*self.num_robot)

		######tesseract environment setup:

		with open("urdf/combined.urdf","r") as f:
			combined_urdf = f.read()
		with open("urdf/combined.srdf","r") as f:
			combined_srdf = f.read()
		t = tesseract.Tesseract()
		t.init(combined_urdf, combined_srdf, GazeboModelResourceLocator())
		self.t_env = t.getEnvironment()
		#update robot poses based on calibration file
		self.t_env.changeJointOrigin("UR1_pose", H_UR1)
		self.t_env.changeJointOrigin("UR2_pose", H_UR2)
		self.t_env.changeJointOrigin("UR3_pose", H_UR3)
		self.t_env.changeJointOrigin("UR4_pose", H_UR4)
		self.t_env.changeJointOrigin("ABB1_pose", H_ABB1)
		self.t_env.changeJointOrigin("ABB2_pose", H_ABB2)
		self.t_env.changeJointOrigin("ABB3_pose", H_ABB3)
		self.t_env.changeJointOrigin("ABB4_pose", H_ABB4)

		contact_distance=1.5
		monitored_link_names = self.t_env.getLinkNames()
		self.manager = self.t_env.getDiscreteContactManager()
		self.manager.setActiveCollisionObjects(monitored_link_names)
		self.manager.setContactDistanceThreshold(contact_distance)

	def start(self):
		self._running=True
		self._camera = threading.Thread(target=self.distance_check_robot)
		self._camera.daemon = True
		self._camera.start()
	def close(self):
		self._running = False
		self._camera.join()

	def distance_check_robot(self):
		while self._running:
			with self._lock:
				try:
					self.L2C=[''] * self.num_robot
					#reset distance matrix
					distance_matrix=-np.ones((self.num_robot,self.num_robot))
					#update all robot joints
					for i in range(self.num_robot):
						robot_joints=self.robot_state_list[i].InValue.joint_position
						self.t_env.setState(self.robot_joint_list[i], robot_joints)
					#get distance check
					env_state = self.t_env.getCurrentState()
					self.manager.setCollisionObjectsTransform(env_state.link_transforms)
					contacts = self.manager.contactTest(2)
					contact_vector = tesseract.flattenResults(contacts)
					distances = np.array([c.distance for c in contact_vector])
					nearest_points=np.array([c.nearest_points for c in contact_vector])
					names = np.array([c.link_names for c in contact_vector])
					for i in range(self.num_robot):
						for j in range(i+1,self.num_robot):
							min_distance=1
							# min_distance_index=0
							for m in range(len(distances)):
								if 	(names[m][0] in self.robot_link_list[i] and names[m][1] in self.robot_link_list[j]) and distances[m]<min_distance:
									min_distance=distances[m]
									self.L2C[i]=names[m][0]
								elif(names[m][0] in self.robot_link_list[j] and names[m][1] in self.robot_link_list[i]) and distances[m]<min_distance:
									min_distance=distances[m]
									self.L2C[i]=names[m][1]

							#update distance matrix
							if min_distance!=1:
								distance_matrix[i][j]=min_distance
								distance_matrix[j][i]=min_distance
					self.distance_matrix=distance_matrix.flatten()
				except:
					traceback.print_exc()

	def distance_check(self,robot_idx):
		with self._lock:
			distance_report=RRN.GetStructureType("edu.rpi.robotics.distance.distance_report")
			distance_report1=distance_report()

			for i in range(self.num_robot):
				robot_joints=self.robot_state_list[i].InValue.joint_position
				self.t_env.setState(self.robot_joint_list[i], robot_joints)

			env_state = self.t_env.getCurrentState()
			self.manager.setCollisionObjectsTransform(env_state.link_transforms)
			contacts = self.manager.contactTest(2)

			contact_vector = tesseract.flattenResults(contacts)

			distances = np.array([c.distance for c in contact_vector])
			nearest_points=np.array([c.nearest_points for c in contact_vector])
			names = np.array([c.link_names for c in contact_vector])
			# nearest_index=np.argmin(distances)
			# print(names)

			min_distance=9
			min_index=0
			Closest_Pt=[0.,0.,0.]
			Closest_Pt_env=[0.,0.,0.]
			#initialize
			distance_report1.Closest_Pt=Closest_Pt
			distance_report1.Closest_Pt_env=Closest_Pt_env

				
			for i in range(len(distances)):
				#only 1 in 2 collision "objects"
				if (names[i][0] in self.robot_link_list[robot_idx] or names[i][1] in self.robot_link_list[robot_idx]) and distances[i]<min_distance and not (names[i][0] in self.robot_link_list[robot_idx] and names[i][1] in self.robot_link_list[robot_idx]):
					min_distance=distances[i]
					min_index=i

			J2C=0
			if (len(distances)>0):
				if names[min_index][0] in self.robot_link_list[robot_idx] and names[min_index][1] in self.robot_link_list[robot_idx]:
					stop=1
				elif names[min_index][0] in self.robot_link_list[robot_idx]:
					J2C=self.robot_link_list[robot_idx].index(names[min_index][0])
					Closest_Pt=nearest_points[min_index][0]
					Closest_Pt_env=nearest_points[min_index][1]
					print(names[min_index])
					print(distances[min_index])
				elif names[min_index][1] in self.robot_link_list[robot_idx]:
					J2C=self.robot_link_list[robot_idx].index(names[min_index][1])
					Closest_Pt=nearest_points[min_index][1]
					Closest_Pt_env=nearest_points[min_index][0]
					print(names[min_index])
					print(distances[min_index])

				distance_report1.Closest_Pt=np.float16(Closest_Pt).flatten().tolist()
				distance_report1.Closest_Pt_env=np.float16(Closest_Pt_env).flatten().tolist()
				distance_report1.min_distance=np.float16(distances[min_index])
				distance_report1.J2C=J2C
				
				
				return distance_report1

			return distance_report1

with RR.ServerNodeSetup("Distance_Service", 25522) as node_setup:
	#register service file and service
	RRN.RegisterServiceTypeFromFile("../../robdef/edu.rpi.robotics.distance")
	distance_inst=create_impl()				#create obj
	distance_inst.start()
	RRN.RegisterService("Environment","edu.rpi.robotics.distance.env",distance_inst)
	print("distance service started")

	# time.sleep(1)	
	# print(distance_inst.distance_matrix.reshape(distance_inst.num_robot,distance_inst.num_robot))


	input("Press enter to quit")









