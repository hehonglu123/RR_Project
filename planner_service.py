#!/usr/bin/env python3
from tesseract.tesseract_scene_graph import SimpleResourceLocator, SimpleResourceLocatorFn
from tesseract.tesseract_environment import Environment
from tesseract.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond
from tesseract.tesseract_collision import ContactResultMap, ContactRequest, ContactTestType_ALL, ContactResultVector
from tesseract.tesseract_collision import flattenResults as collisionFlattenResults
from tesseract_viewer import TesseractViewer
import os, re, copy
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
import numpy as np
from qpsolvers import solve_qp
import yaml, time, traceback, threading, sys
sys.path.append('../')
from gazebo_model_resource_locator import GazeboModelResourceLocator
sys.path.append('toolbox')
from abb_ik import inv as inv_abb
from sawyer_ik import inv as inv_sawyer
from ur_ik import inv as inv_ur

from general_robotics_toolbox import *    

def normalize_dq(q):
	q[:-1]=0.5*q[:-1]/(np.linalg.norm(q[:-1])) 
	return q  
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
		with open('calibration/Sawyer.yaml') as file:
			H_Sawyer 	= np.array(yaml.load(file)['H'],dtype=np.float64)
		with open('calibration/ur.yaml') as file:
			H_UR 		= np.array(yaml.load(file)['H'],dtype=np.float64)
		with open('calibration/abb.yaml') as file:
			H_ABB 	= np.array(yaml.load(file)['H'],dtype=np.float64)

		self.H_UR=H42H3(H_UR)
		self.H_Sawyer=H42H3(H_Sawyer)
		self.H_ABB=H42H3(H_ABB)
		self.H_robot={'ur':self.H_UR,'sawyer':self.H_Sawyer,'abb':self.H_ABB}
		

		self.distance_report=RRN.GetStructureType("edu.rpi.robotics.distance.distance_report")
		self.dict={'ur':0,'sawyer':1,'abb':2}
		self.distance_report_dict={}
		for robot_name,robot_idx in self.dict.items():
			self.distance_report_dict[robot_name]=self.distance_report()


		#form H into RR transformation struct
		self.transformations={}
		self.transformation=RRN.GetStructureType("edu.rpi.robotics.distance.transformation")

		transformation1=self.transformation()
		transformation1.name="UR"
		transformation1.row=len(self.H_UR)
		transformation1.column=len(self.H_UR[0])
		transformation1.H=np.float16(self.H_UR).flatten().tolist()
		self.transformations['ur']=transformation1

		transformation2=self.transformation()
		transformation2.name="Sawyer"
		transformation2.row=len(self.H_Sawyer)
		transformation2.column=len(self.H_Sawyer[0])
		transformation2.H=np.float16(self.H_Sawyer).flatten().tolist()
		self.transformations['sawyer']=transformation2

		transformation3=self.transformation()
		transformation3.name="ABB"
		transformation3.row=len(self.H_ABB)
		transformation3.column=len(self.H_ABB[0])
		transformation3.H=np.float16(self.H_ABB).flatten().tolist()
		self.transformations['abb']=transformation3

		
		

		#Connect to robot service
		with open('client_yaml/client_ur.yaml') as file:
			self.url_ur= yaml.load(file)['url']
		with open('client_yaml/client_sawyer.yaml') as file:
			self.url_sawyer= yaml.load(file)['url']
		with open('client_yaml/client_abb.yaml') as file:
			self.url_abb= yaml.load(file)['url']

		self.ur_sub=RRN.SubscribeService(self.url_ur)
		self.sawyer_sub=RRN.SubscribeService(self.url_sawyer)
		self.abb_sub=RRN.SubscribeService(self.url_abb)

		self.ur_sub.ClientConnectFailed += self.connect_failed
		self.sawyer_sub.ClientConnectFailed += self.connect_failed
		self.abb_sub.ClientConnectFailed += self.connect_failed

		UR_state=self.ur_sub.SubscribeWire("robot_state")
		Sawyer_state=self.sawyer_sub.SubscribeWire("robot_state")
		ABB_state=self.abb_sub.SubscribeWire("robot_state")


		#link and joint names in urdf
		Sawyer_joint_names=["right_j0","right_j1","right_j2","right_j3","right_j4","right_j5","right_j6"]
		UR_joint_names=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
		Sawyer_link_names=["right_l0","right_l1","right_l2","right_l3","right_l4","right_l5","right_l6","right_l1_2","right_l2_2","right_l4_2","right_hand"]
		UR_link_names=['UR_base_link',"shoulder_link","upper_arm_link","forearm_link","wrist_1_link","wrist_2_link","wrist_3_link","UR_link_7"]
		ABB_joint_names=['ABB1200_joint_1','ABB1200_joint_2','ABB1200_joint_3','ABB1200_joint_4','ABB1200_joint_5','ABB1200_joint_6']
		ABB_link_names=['ABB1200_base_link','ABB1200_link_1','ABB1200_link_2','ABB1200_link_3','ABB1200_link_4','ABB1200_link_5','ABB1200_link_6','ABB1200_link_7']
	
		self.robot_state_list=[UR_state,Sawyer_state,ABB_state]
		self.robot_link_list=[UR_link_names,Sawyer_link_names,ABB_link_names]
		self.robot_joint_list=[UR_joint_names,Sawyer_joint_names,ABB_joint_names]
		self.num_robot=len(self.robot_state_list)

		######tesseract environment setup:
		self.t_env = Environment()
		urdf_path = FilesystemPath("urdf/combined.urdf")
		srdf_path = FilesystemPath("urdf/combined.srdf")
		assert self.t_env.init(urdf_path, srdf_path, GazeboModelResourceLocator())

		#update robot poses based on calibration file
		self.t_env.changeJointOrigin("ur_pose", Isometry3d(H_UR))
		self.t_env.changeJointOrigin("sawyer_pose", Isometry3d(H_Sawyer))
		self.t_env.changeJointOrigin("abb_pose", Isometry3d(H_ABB))

		contact_distance=0.2
		monitored_link_names = self.t_env.getLinkNames()
		self.manager = self.t_env.getDiscreteContactManager()
		self.manager.setActiveCollisionObjects(monitored_link_names)
		self.manager.setContactDistanceThreshold(contact_distance)
		# viewer update
		self.viewer = TesseractViewer()
		self.viewer.update_environment(self.t_env, [0,0,0])
		self.viewer.start_serve_background()


		self.robot_def_dict={}
		try:
			UR=self.ur_sub.GetDefaultClientWait(1)
			self.robot_def_dict['ur']=Robot(np.transpose(np.array(UR.robot_info.chains[0].H.tolist())),np.transpose(np.array(UR.robot_info.chains[0].P.tolist())),np.zeros(len(UR.robot_info.joint_info)))
		except:
			pass
		try:
			Sawyer=self.sawyer_sub.GetDefaultClientWait(1)
			self.robot_def_dict['sawyer']=Robot(np.transpose(np.array(Sawyer.robot_info.chains[0].H.tolist())),np.transpose(np.array(Sawyer.robot_info.chains[0].P.tolist())),np.zeros(len(Sawyer.robot_info.joint_info)))
		except:
			pass

		try:
			ABB=self.abb_sub.GetDefaultClientWait(1)
			self.robot_def_dict['abb']=Robot(np.transpose(np.array(ABB.robot_info.chains[0].H.tolist())),np.transpose(np.array(ABB.robot_info.chains[0].P.tolist())),np.zeros(len(ABB.robot_info.joint_info)))
		except:
			pass
			

		#trajectories
		self.traj_change=False
		self.traj_change_name=None
		self.steps=300
		self.plan_time=0.15
		self.execution_delay=0.03
		self.trajectory={'ur':np.zeros((self.steps,7)),'sawyer':np.zeros((self.steps,8)),'abb':np.zeros((self.steps,7))}
		self.traj_joint_names={'ur':['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
		'sawyer':['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6'],
		'abb':['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
		}
		self.time_step=0.03
		#initialize static trajectories
		for key, value in self.trajectory.items():
			for i in range(self.steps):
				try:
					value[i]=np.append([0],self.robot_state_list[self.dict[key]].InValue.joint_position)
				except:	#incase robot not on
					value[i]=np.append([0],[0]*len(self.robot_joint_list[self.dict[key]]))
		self.inv={'ur':inv_ur,'sawyer':inv_sawyer,'abb':inv_abb}
		self.joint_names_traj={'ur':inv_ur,'sawyer':inv_sawyer,'abb':inv_abb}

		

		#register service constant
		self.JointTrajectoryWaypoint = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint")
		self.JointTrajectory = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectory")

	#connection failed callback
	def connect_failed(self, s, client_id, url, err):
		print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))
		if url==self.url_ur:
			self.ur_sub=RRN.SubscribeService(self.url_ur)
		elif url==self.url_sawyer:
			self.sawyer_sub=RRN.SubscribeService(self.url_sawyer)

	def Sawyer_link(self,J2C):
		if J2C+1==7:
			return 1
		elif J2C+1==8:
			return 2
		elif J2C+1==9:
			return 4
		elif J2C+1==10:
			return 7
		else:
			return J2C

	def viewer_update(self):
		while self._running:
			with self._lock:
				for i in range(self.num_robot):
					robot_joints=self.robot_state_list[i].InValue.joint_position
					self.t_env.setState(self.robot_joint_list[i], robot_joints)
				self.viewer.update_environment(self.t_env, [0,0,0])

	def distance_check_global(self,robot_name, joints_list):
		with self._lock:
			robot_idx=self.dict[robot_name]
			distance_report=RRN.GetStructureType("edu.rpi.robotics.distance.distance_report")
			distance_report1=distance_report()

			for i in range(self.num_robot):
				robot_joints=copy.deepcopy(joints_list[i])
				if i==0:
					robot_joints[0]+=np.pi 		#UR configuration
				self.t_env.setState(self.robot_joint_list[i], robot_joints)

			env_state = self.t_env.getCurrentState()
			self.manager.setCollisionObjectsTransform(env_state.link_transforms)

			result = ContactResultMap()

			self.manager.contactTest(result, ContactRequest(ContactTestType_ALL))
			result_vector = ContactResultVector()
			collisionFlattenResults(result,result_vector)

			distances = [r.distance for r in result_vector]
			nearest_points=[[r.nearest_points[0],r.nearest_points[1]] for r in result_vector]

			names = [[r.link_names[0],r.link_names[1]] for r in result_vector]
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

				elif names[min_index][0] in self.robot_link_list[robot_idx]:
					J2C=self.robot_link_list[robot_idx].index(names[min_index][0])-1
					Closest_Pt=nearest_points[min_index][0]
					Closest_Pt_env=nearest_points[min_index][1]

				elif names[min_index][1] in self.robot_link_list[robot_idx]:
					J2C=self.robot_link_list[robot_idx].index(names[min_index][1])-1
					Closest_Pt=nearest_points[min_index][1]
					Closest_Pt_env=nearest_points[min_index][0]


				if robot_idx==1:
					J2C=self.Sawyer_link(J2C)

				
				distance_report1.Closest_Pt=np.float16(Closest_Pt).flatten().tolist()
				distance_report1.Closest_Pt_env=np.float16(Closest_Pt_env).flatten().tolist()
				distance_report1.min_distance=np.float16(distances[min_index])
				distance_report1.J2C=(J2C if J2C>=0 else 0)		
				
				
				return distance_report1

			return distance_report1

	def plan(self,robot_name,speed,pd,Rd,joint_threshold, obj_vel, capture_time):            #start and end configuration in joint space

		plan_start_time=time.time()
		traj_start_time=time.time()+self.plan_time+self.execution_delay
		#tracking param
		inv_time_check=0.
		#update other robot static trajectories
		for key, value in self.trajectory.items():
			#only for ones not moving
			if value[0][0]==0:
				try:
					value[:]=np.append([0],self.robot_state_list[self.dict[key]].InValue.joint_position)
				except:
					value[:]=np.append([0],[0]*len(self.robot_joint_list[self.dict[key]]))


		Rd=Rd.reshape((3,3))
		
		distance_threshold=0.1

		#parameter setup
		n= len(self.robot_joint_list[self.dict[robot_name]])

		#calc desired joint angles
		q_des=self.inv[robot_name](pd,Rd).reshape(n)


		w=1                  #set the weight between orientation and position
		Kq=.01*np.eye(n)    #small value to make sure positive definite
		Kp=np.eye(3)
		KR=np.eye(3)        #gains for position and orientation error
		step=0

		EP=[1,1,1]
		q_cur=self.robot_state_list[self.dict[robot_name]].InValue.joint_position

		#initialize trajectory
		self.trajectory[robot_name][step]=np.append(0.,q_cur)
		waypoints = []
		wp = self.JointTrajectoryWaypoint()
		wp.joint_position = copy.deepcopy(q_cur)
		wp.time_from_start = 0.
		waypoints.append(wp)

		#get joint info in future 
		other_robot_trajectory_start_idx={'ur':0,'sawyer':0,'abb':0}
		for key, value in self.trajectory.items():
			if key==robot_name:
				continue
			if value[0][0]!=0:
				other_robot_trajectory_start_idx[key] = (np.abs(value[:,0] - traj_start_time)).argmin()

		while(np.linalg.norm(q_des[:-1]-q_cur[:-1])>joint_threshold):

			#in case getting stuck
			if step>self.steps:
				raise UnboundLocalError("Unplannable")
				return 
			
			if np.linalg.norm(obj_vel)!=0 and step*self.time_step-inv_time_check>0.2:
				p_d=(pd+obj_vel*(step*self.time_step+self.execution_delay+0.2))
				try:

					q_des=self.inv[robot_name](p_d,Rd).reshape(n)
					inv_time_check=step*self.time_step

				except:
					raise UnboundLocalError
					return
			else:
				p_d=pd

			#if trajectory of other robot changed
			if self.traj_change:
				if self.traj_change_name!=robot_name:
					other_robot_trajectory_start_idx[self.traj_change_name] = (np.abs(self.trajectory[self.traj_change_name][:,0] - traj_start_time-step*self.time_step)).argmin()-step
					self.traj_change=False
					self.traj_change_name=None
					self.clear_traj(robot_name)
					raise AttributeError("Trajectory change")
					return
		#     get current H and J
			robot_pose=self.robot_state_list[self.dict[robot_name]].InValue.kin_chain_tcp[0]
			R_cur = q2R(np.array(robot_pose['orientation'].tolist()))

			p_cur=np.array(robot_pose['position'].tolist())

			if robot_name=='ur':
				q_temp=copy.deepcopy(q_cur) 
				q_temp[0]+=np.pi                        #UR configuration
				J=robotjacobian(self.robot_def_dict[robot_name],q_temp)        #calculate current Jacobian
			else:    
				J=robotjacobian(self.robot_def_dict[robot_name],q_cur)        #calculate current Jacobian
			Jp=J[3:,:]
			JR=J[:3,:]                              #decompose to position and orientation Jacobian

			ER=np.dot(R_cur,np.transpose(Rd))
			EP=p_cur-p_d                             #error in position and orientation
			#update future joint to distance checking

			joints_list=[self.trajectory['ur'][np.amin([step+other_robot_trajectory_start_idx['ur'],self.steps-1])][1:],self.trajectory['sawyer'][np.amin([step+other_robot_trajectory_start_idx['sawyer'],self.steps-1])][1:],
			self.trajectory['abb'][np.amin([step+other_robot_trajectory_start_idx['abb'],self.steps-1])][1:]]
			#update self joint position
			joints_list[self.dict[robot_name]]=q_cur
			distance_report=self.distance_check_global(robot_name,joints_list)


			Closest_Pt=distance_report.Closest_Pt
			Closest_Pt_env=distance_report.Closest_Pt_env
			dist=distance_report.min_distance
			J2C=distance_report.J2C

			if (Closest_Pt[0]!=0. and dist<distance_threshold) and J2C>2: 
				

				# print("qp triggering ",dist )
				Closest_Pt[:2]=np.dot(self.H_robot[robot_name],np.append(Closest_Pt[:2],1))[:2]
				Closest_Pt_env[:2]=np.dot(self.H_robot[robot_name],np.append(Closest_Pt_env[:2],1))[:2] 

				k,theta = R2rot(ER)             #decompose ER to (k,theta) pair

			#   set up s for different norm for ER

				s=np.sin(theta/2)*k         #eR2
				vd=-np.dot(Kp,EP)
				wd=-np.dot(KR,s)          
				H=np.dot(np.transpose(Jp),Jp)+Kq+w*np.dot(np.transpose(JR),JR)
				H=(H+np.transpose(H))/2

				f=-np.dot(np.transpose(Jp),vd)-w*np.dot(np.transpose(JR),wd)               #setup quadprog parameters


				dx = Closest_Pt_env[0] - Closest_Pt[0]
				dy = Closest_Pt_env[1] - Closest_Pt[1]
				dz = Closest_Pt_env[2] - Closest_Pt[2]

				# derivative of dist w.r.t time
				der = np.array([dx/dist, dy/dist, dz/dist])
				J_Collision=np.hstack((J[3:,:J2C],np.zeros((3,n-J2C))))

				A=np.dot(der.reshape((1,3)),J_Collision)
				
				b=np.array([dist - 0.1])

				try:
					qdot=normalize_dq(solve_qp(H, f,A,b))
					
				except:
					traceback.print_exc()

			else:
				qdot=normalize_dq(q_des-q_cur)
				#accelerate within 1st second
				if (step+1)*self.time_step<.5:
					qdot*=(speed*(step+1)*self.time_step/.5)
				elif np.linalg.norm(q_des-q_cur)>0.4:
					qdot*=speed
				else:
					qdot*=(speed*np.linalg.norm(q_des-q_cur)/0.4)
			qdot[-1]=q_des[-1]-q_cur[-1]
			#update q_cur
			qdot[-1]=np.amin([qdot[-1],3.])
			qdot[-1]=np.amax([qdot[-1],-3.])

			q_cur+=qdot*self.time_step
			step+=1
			self.trajectory[robot_name][step]=np.append(self.time_step*step,q_cur)
			#RR trajectory formation
			
			wp = self.JointTrajectoryWaypoint()
			wp.joint_position = copy.deepcopy(q_cur)
			wp.time_from_start = step*self.time_step
			waypoints.append(wp)

		
		#populate all after the goal configuration
		self.trajectory[robot_name][step:]=np.append(self.time_step*step,q_cur)

		traj = self.JointTrajectory()
		traj.joint_names = self.traj_joint_names[robot_name]
		traj.waypoints = waypoints

		#dynamic planning time
		self.plan_time=time.time()-plan_start_time

		#estimate of trajectory timestamp+prox execution delay
		self.trajectory[robot_name][:,0]+=time.time()+self.execution_delay

		self.traj_change_name=robot_name
		self.traj_change=True


		return traj
	def clear_traj(self,robot_name):
		#clear trajectory after execution
		self.trajectory[robot_name][:]=np.append([0],self.robot_state_list[self.dict[robot_name]].InValue.joint_position)

with RR.ServerNodeSetup("Distance_Service", 25522) as node_setup:
	cwd = os.getcwd()
	#register robot service definition
	RRC. RegisterStdRobDefServiceTypes(RRN)

	#register service file and service
	RRN.RegisterServiceTypeFromFile("robdef/edu.rpi.robotics.distance")
	distance_inst=create_impl()				#create obj

	RRN.RegisterService("Environment","edu.rpi.robotics.distance.env",distance_inst)

	# distance_inst.plan('abb',2.,[0.5,0.5,0.5],np.array([1,0,0,0,1,0,0,0,1]),0.1,[0,0,0],0)
	input("Press enter to quit")









