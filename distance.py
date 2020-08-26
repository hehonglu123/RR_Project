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

#resource locator class
class TesseractSupportResourceLocator(tesseract.ResourceLocator):
	def __init__(self):
		super(TesseractSupportResourceLocator,self).__init__()
		self.TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]

	def locateResource(self, url):
		try:
			url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$",url)
			if (url_match is None):
				return None
			
			fname = os.path.join(self.TESSERACT_SUPPORT_DIR, os.path.normpath(url_match.group(1)))
			with open(fname,'rb') as f:
				resource_bytes = f.read()

			resource = tesseract.BytesResource(url, resource_bytes)

			return resource
		except:
			traceback.print_exc()
			return


class create_impl(object):
	def __init__(self):
		self._lock=threading.RLock()
		#load calibration parameters
		with open('calibration/UR.yaml') as file:
			self.H_UR 		= np.array(yaml.load(file)['H'])
		with open('calibration/Sawyer.yaml') as file:
			self.H_Sawyer 	= np.array(yaml.load(file)['H'])

		#form H into RR transformation struct
		transformation=RRN.GetStructureType("edu.rpi.robotics.distance.transformation")
		transformation1=transformation()
		transformation1.name="UR"
		transformation1.row=len(self.H_UR)
		transformation1.column=len(self.H_UR[0])
		transformation1.H=np.float16(self.H_UR).flatten().tolist()

		transformation2=transformation()
		transformation2.name="Sawyer"
		transformation2.row=len(self.H_Sawyer)
		transformation2.column=len(self.H_Sawyer[0])
		transformation2.H=np.float16(self.H_Sawyer).flatten().tolist()
		self.transformations=[transformation1,transformation2]

		#Connect to robot service
		self.UR = RRN.ConnectService('rr+tcp://localhost:58652?service=ur_robot')
		self.Sawyer= RRN.ConnectService('rr+tcp://127.0.0.1:58653?service=sawyer')
		self.UR_state=self.UR.robot_state.Connect()
		self.Sawyer_state=self.Sawyer.robot_state.Connect()

		#link and joint names in urdf
		self.Sawyer_joint_names=["right_j0","right_j1","right_j2","right_j3","right_j4","right_j5","right_j6"]
		self.UR_joint_names=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
		self.Sawyer_link_names=["right_l0","right_l1","right_l2","right_l3","right_l4","right_l5","right_l6","right_l1_2","right_l2_2","right_l4_2","right_hand"]
		self.UR_link_names=["shoulder_link","upper_arm_link","forearm_link","wrist_1_link","wrist_2_link","wrist_3_link","ee_link"]



		######tesseract environment setup:
		TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]
		with open("urdf/combined.urdf",'r') as f:
			combined_urdf = f.read()
		with open("urdf/combined.srdf",'r') as f:
			combined_srdf = f.read()
		t = tesseract.Tesseract()
		t.init(combined_urdf, combined_srdf, TesseractSupportResourceLocator())
		self.t_env = t.getEnvironment()
		contact_distance=0.11
		monitored_link_names = self.t_env.getLinkNames()
		self.manager = self.t_env.getDiscreteContactManager()
		self.manager.setActiveCollisionObjects(monitored_link_names)
		self.manager.setContactDistanceThreshold(contact_distance)

	def get_joint_angle(self,robot_state):
		return robot_state.InValue.joint_position


	def distance_check(self,robot_name):
		with self._lock:
			distance_report=RRN.GetStructureType("edu.rpi.robotics.distance.distance_report")
			distance_report1=distance_report()

			#get current joint angles
			UR_joints=self.get_joint_angle(self.UR_state)
			Sawyer_joints=self.get_joint_angle(self.Sawyer_state)


			UR_joints[0]+=np.pi 		#bias
			#update environment with current joints

			try:
				self.t_env.setState(self.Sawyer_joint_names, Sawyer_joints)
				self.t_env.setState(self.UR_joint_names, UR_joints)
			except:
				traceback.print_exc()
			env_state = self.t_env.getCurrentState()
			self.manager.setCollisionObjectsTransform(env_state.transforms)
			contacts = self.manager.contactTest(2)

			contact_vector = tesseract.flattenResults(contacts)

			distances = np.array([c.distance for c in contact_vector])
			nearest_points=np.array([c.nearest_points for c in contact_vector])
			names = np.array([c.link_names for c in contact_vector])
			# nearest_index=np.argmin(distances)
			# print(names)

			min_UR=9
			min_UR_index=0
			min_Sawyer=9
			min_Sawyer_index=0
			Closest_Pt=[0.,0.,0.]
			Closest_Pt_env=[0.,0.,0.]
			#initialize
			distance_report1.Closest_Pt=Closest_Pt
			distance_report1.Closest_Pt_env=Closest_Pt_env

			if(robot_name=="UR"):
				
				for i in range(len(distances)):
					if (names[i][0] in self.UR_link_names or names[i][1] in self.UR_link_names) and distances[i]<min_UR and not (names[i][0] in self.UR_link_names and names[i][1] in self.UR_link_names):
						min_UR=distances[i]
						min_UR_index=i

				UR_link_name="nothing"
				if (len(distances)>0):
					if names[min_UR_index][0] in self.UR_link_names and names[min_UR_index][1] in self.UR_link_names:
						stop=1
					elif names[min_UR_index][0] in self.UR_link_names:
						UR_link_name=names[min_UR_index][0]
						Closest_Pt=nearest_points[min_UR_index][0]
						Closest_Pt_env=nearest_points[min_UR_index][1]
						print(names[min_UR_index])
						print(distances[min_UR_index])
					elif names[min_UR_index][1] in self.UR_link_names:
						UR_link_name=names[min_UR_index][1]
						Closest_Pt=nearest_points[min_UR_index][1]
						Closest_Pt_env=nearest_points[min_UR_index][0]
						print(names[min_UR_index])
						print(distances[min_UR_index])

					distance_report1.Closest_Pt=np.float16(Closest_Pt).flatten().tolist()
					distance_report1.Closest_Pt_env=np.float16(Closest_Pt).flatten().tolist()
					distance_report1.min_distance=np.float16(distances[min_UR_index])
					distance_report1.robot_link_name=UR_link_name
					
					
					return distance_report1
			elif(robot_name=="Sawyer"):

				for i in range(len(distances)):
					if (names[i][0] in self.Sawyer_link_names or names[i][1] in self.Sawyer_link_names) and distances[i]<min_Sawyer and not (names[i][0] in self.Sawyer_link_names and names[i][1] in self.Sawyer_link_names):
						min_Sawyer=distances[i]
						min_Sawyer_index=i
				Sawyer_link_name="nothing"
				if (len(distances)>0):

					if names[min_Sawyer_index][0] in self.Sawyer_link_names and names[min_Sawyer_index][1] in self.Sawyer_link_names:
						stop=1
					elif names[min_Sawyer_index][0] in self.Sawyer_link_names:
						Sawyer_link_name=names[min_Sawyer_index][0]
						Closest_Pt=nearest_points[min_Sawyer_index][0]
						Closest_Pt_env=nearest_points[min_Sawyer_index][1]
						print(names[min_Sawyer_index])
						print(distances[min_Sawyer_index])
					elif names[min_Sawyer_index][1] in self.Sawyer_link_names:
						Sawyer_link_name=names[min_Sawyer_index][1]
						Closest_Pt=nearest_points[min_Sawyer_index][1]
						Closest_Pt_env=nearest_points[min_Sawyer_index][0]
						print(names[min_Sawyer_index])
						print(distances[min_Sawyer_index])

						
					distance_report1.Closest_Pt=np.float16(Closest_Pt).flatten().tolist()
					distance_report1.Closest_Pt_env=np.float16(Closest_Pt).flatten().tolist()
					distance_report1.min_distance=np.float16(distances[min_Sawyer_index])
					distance_report1.robot_link_name=Sawyer_link_name
					
					
					return distance_report1
				
			return distance_report1



with RR.ServerNodeSetup("Distance_Service", 25522) as node_setup:
	#register service file and service
	RRN.RegisterServiceTypeFromFile("robdef/edu.rpi.robotics.distance")
	distance_inst=create_impl()				#create obj
	RRN.RegisterService("Environment","edu.rpi.robotics.distance.env",distance_inst)


	
	
	# print(distance_inst.distance_check("Sawyer"))



	input("Press enter to quit")









