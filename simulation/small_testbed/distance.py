#!/usr/bin/env python3
import tesseract
from tesseract_viewer import TesseractViewer
import os, re
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np
import yaml, time, traceback, threading, sys
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
		#load calibration parameters
		with open('calibration/rp260.yaml') as file:
			H_rp260 	= np.array(yaml.load(file)['H'],dtype=np.float64)

		self.H_rp260=H42H3(H_rp260)

		self.L2C=['']

		self.dict={'rp260':0}

		#connect to RR gazebo plugin service
		server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
		self.w=server.get_worlds(str(server.world_names[0]))
		#create RR pose type
		pose_dtype = RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose", server)
		self.model_pose = np.zeros((1,), dtype = pose_dtype)

		#form H into RR transformation struct
		self.transformations={}
		self.transformation=RRN.GetStructureType("edu.rpi.robotics.distance.transformation")

		transformation1=self.transformation()
		transformation1.name="rp260"
		transformation1.row=len(self.H_rp260)
		transformation1.column=len(self.H_rp260[0])
		transformation1.H=np.float16(self.H_rp260).flatten().tolist()
		self.transformations['rp260']=transformation1
		
		
		

		#Connect to robot service
		rp260 = RRN.ConnectService('rr+tcp://localhost:23333?service=robot')
		rp260_state=rp260.robot_state.Connect()


		#link and joint names in urdf

		rp260_joint_names=['rp260__joint_1','rp260__joint_2','rp260__joint_3','rp260__joint_4','rp260__joint_5','rp260__joint_6']
		rp260_link_names=['rp260__base_link','rp260__link_1','rp260__link_2','rp260__link_3','rp260__link_4','rp260__link_5','rp260__link_6']

		self.robot_state_list=[rp260_state]
		self.robot_link_list=[rp260_link_names]
		self.robot_joint_list=[rp260_joint_names]
		self.num_robot=len(self.robot_state_list)
		self.distance_matrix=-np.ones(self.num_robot*self.num_robot)

		######tesseract environment setup:

		with open("urdf/combined.urdf",'r') as f:
			combined_urdf = f.read()
		with open("urdf/combined.srdf",'r') as f:
			combined_srdf = f.read()
		t = tesseract.Tesseract()
		t.init(combined_urdf, combined_srdf, GazeboModelResourceLocator())
		self.t_env = t.getEnvironment()
		#update robot poses based on calibration file
		self.t_env.changeJointOrigin("rp260_pose", H_rp260)


		contact_distance=0.2
		monitored_link_names = self.t_env.getLinkNames()
		self.manager = self.t_env.getDiscreteContactManager()
		self.manager.setActiveCollisionObjects(monitored_link_names)
		self.manager.setContactDistanceThreshold(contact_distance)
		# viewer update
		self.viewer = TesseractViewer()
		self.viewer.update_environment(self.t_env, [0,0,0])
		self.viewer.start_serve_background()


	def Sawyer_link(self,J2C):
		if J2C==7:
			return 1
		elif J2C==8:
			return 2
		elif J2C==9:
			return 4
		elif J2C==10:
			return 7
		else:
			return J2C


	def start(self):
		self._running=True
		self._camera = threading.Thread(target=self.viewer_update)
		self._camera.daemon = True
		self._camera.start()
	def close(self):
		self._running = False
		self._camera.join()

	def viewer_update(self):
		while self._running:
			with self._lock:
				for i in range(self.num_robot):
					robot_joints=self.robot_state_list[i].InValue.joint_position
					self.t_env.setState(self.robot_joint_list[i], robot_joints)
				self.viewer.update_environment(self.t_env, [0,0,0])

	def distance_check_robot(self):
		while self._running:
			with self._lock:
				try:
					self.L2C=['','','','']
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

	def distance_check(self,robot_name):
		with self._lock:
			robot_idx=self.dict[robot_name]
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

			

			min_distance=9
			min_index=-1
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
			if (min_index!=-1):
				if names[min_index][0] in self.robot_link_list[robot_idx] and names[min_index][1] in self.robot_link_list[robot_idx]:
					stop=1
					print("stop")
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

				if robot_idx==1:
					J2C=self.Sawyer_link(J2C)

				distance_report1.Closest_Pt=np.float16(Closest_Pt).flatten().tolist()
				distance_report1.Closest_Pt_env=np.float16(Closest_Pt_env).flatten().tolist()
				distance_report1.min_distance=np.float16(distances[min_index])
				distance_report1.J2C=J2C	
				
				
				return distance_report1

			return distance_report1

	def roll(self,robot_model_name, x, y, z, angle):
		#remove the robot model first
		self.w.remove_model(robot_model_name)

		#initialize new pose 
		self.model_pose["orientation"]['w'] = np.cos(angle/2.)
		self.model_pose["orientation"]['x'] = 0
		self.model_pose["orientation"]['y'] = 0
		self.model_pose["orientation"]['z'] = np.sin(angle/2.)
		self.model_pose["position"]['x']=x
		self.model_pose["position"]['y']=y
		self.model_pose["position"]['z']=z

		#read in the robot sdf
		f = open('../models/'+robot_model_name+'/model.sdf','r')
		robot_sdf = f.read()
		#insert the robot
		time.sleep(.5)
		self.w.insert_model(robot_sdf, robot_model_name, self.model_pose)

		H=np.array([[np.cos(angle),-np.sin(angle),0,x],
					[np.sin(angle), np.cos(angle),0,y],
					[    		 0, 			0,1,z],
					[    		 0, 			0,0,1]],dtype=np.float64)
		####typecast, convert unicode to string
		joint_name=str(robot_model_name+"_pose")
		####change joint origin
		self.t_env.changeJointOrigin(joint_name, H)
		####update viewer
		self.viewer.update_environment(self.t_env, [0,0,0])
		####update transformation in service
		transformation_temp=self.transformation()
		transformation_temp.name=robot_model_name
		H=H42H3(H)
		transformation_temp.row=len(H)
		transformation_temp.column=len(H[0])
		transformation_temp.H=np.float16(H).flatten().tolist()
		self.transformations[robot_model_name]=transformation_temp
		return

with RR.ServerNodeSetup("Distance_Service", 25522) as node_setup:
	#register service file and service
	RRN.RegisterServiceTypeFromFile("../../robdef/edu.rpi.robotics.distance")
	distance_inst=create_impl()				#create obj
	# distance_inst.start()
	RRN.RegisterService("Environment","edu.rpi.robotics.distance.env",distance_inst)
	print("distance service started")

	input("Press enter to quit")
	distance_inst.stop()









