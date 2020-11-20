#!/usr/bin/env python3
import tesseract
from tesseract_viewer import TesseractViewer
import os, re, copy
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np
from qpsolvers import solve_qp
import yaml, time, traceback, threading, sys
sys.path.append('../')
from gazebo_model_resource_locator import GazeboModelResourceLocator
sys.path.append('../../toolbox')
from abb_ik import inv as inv_abb
from sawyer_ik import inv as inv_sawyer
from ur_ik import inv as inv_ur
from staubli_ik import inv as inv_staubli

from general_robotics_toolbox import *    

#convert 4x4 H matrix to 3x3 H matrix and inverse for mapping obj to robot frame
def H42H3(H):
	H3=np.linalg.inv(H[:2,:2])
	H3=np.hstack((H3,-np.dot(H3,np.array([[H[0][-1]],[H[1][-1]]]))))
	H3=np.vstack((H3,np.array([0,0,1])))
	return H3
def normalize_dq(q):
	q[:-1]=0.5*q[:-1]/(np.linalg.norm(q[:-1])) 
	return q   
class create_impl(object):
	def __init__(self):
		self._lock=threading.RLock()
		self._running=False
		#load calibration parameters
		with open('calibration/Sawyer2.yaml') as file:
			H_Sawyer 	= np.array(yaml.load(file)['H'],dtype=np.float64)
		with open('calibration/UR2.yaml') as file:
			H_UR 		= np.array(yaml.load(file)['H'],dtype=np.float64)
		with open('calibration/ABB2.yaml') as file:
			H_ABB 	= np.array(yaml.load(file)['H'],dtype=np.float64)
		with open('calibration/tx60.yaml') as file:
			H_tx60 	= np.array(yaml.load(file)['H'],dtype=np.float64)
		self.H_UR=H42H3(H_UR)
		self.H_Sawyer=H42H3(H_Sawyer)
		self.H_ABB=H42H3(H_ABB)
		self.H_tx60=H42H3(H_tx60)
		self.H_robot={'ur':self.H_UR,'sawyer':self.H_Sawyer,'abb':self.H_ABB,'staubli':self.H_tx60}
		
		self.distance_report=RRN.GetStructureType("edu.rpi.robotics.distance.distance_report")
		self.dict={'ur':0,'sawyer':1,'abb':2,'staubli':3}
		self.distance_report_dict={}
		for robot_name,robot_idx in self.dict.items():
			self.distance_report_dict[robot_name]=self.distance_report()

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

		transformation4=self.transformation()
		transformation4.name="Staubli"
		transformation4.row=len(self.H_tx60)
		transformation4.column=len(self.H_tx60[0])
		transformation4.H=np.float16(self.H_tx60).flatten().tolist()
		self.transformations['staubli']=transformation4
		
		
		

		#Connect to robot service
		UR = RRN.ConnectService('rr+tcp://localhost:58653?service=robot')
		Sawyer= RRN.ConnectService('rr+tcp://localhost:58654?service=robot')
		ABB= RRN.ConnectService('rr+tcp://localhost:58655?service=robot')
		tx60= RRN.ConnectService('rr+tcp://localhost:58656?service=robot')
		UR_state=UR.robot_state.Connect()
		Sawyer_state=Sawyer.robot_state.Connect()
		ABB_state=ABB.robot_state.Connect()
		tx60_state=tx60.robot_state.Connect()

		#link and joint names in urdf
		Sawyer_joint_names=["right_j0","right_j1","right_j2","right_j3","right_j4","right_j5","right_j6"]
		UR_joint_names=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
		Sawyer_link_names=["right_l0","right_l1","right_l2","right_l3","right_l4","right_l5","right_l6","right_l1_2","right_l2_2","right_l4_2","right_hand"]
		UR_link_names=['UR_base_link',"shoulder_link","upper_arm_link","forearm_link","wrist_1_link","wrist_2_link","wrist_3_link"]
		ABB_joint_names=['ABB1200_joint_1','ABB1200_joint_2','ABB1200_joint_3','ABB1200_joint_4','ABB1200_joint_5','ABB1200_joint_6']
		ABB_link_names=['ABB1200_base_link','ABB1200_link_1','ABB1200_link_2','ABB1200_link_3','ABB1200_link_4','ABB1200_link_5','ABB1200_link_6']
		tx60_joint_names=['tx60_joint_1','tx60_joint_2','tx60_joint_3','tx60_joint_4','tx60_joint_5','tx60_joint_6']
		tx60_link_names=['tx60_base_link','tx60_link_1','tx60_link_2','tx60_link_3','tx60_link_4','tx60_link_5','tx60_link_6']

		self.robot_list=[UR,Sawyer,ABB,tx60]
		self.robot_state_list=[UR_state,Sawyer_state,ABB_state,tx60_state]
		self.robot_link_list=[UR_link_names,Sawyer_link_names,ABB_link_names,tx60_link_names]
		self.robot_joint_list=[UR_joint_names,Sawyer_joint_names,ABB_joint_names,tx60_joint_names]
		self.num_robot=len(self.robot_state_list)

		######tesseract environment setup:

		with open("urdf/combined.urdf",'r') as f:
			combined_urdf = f.read()
		with open("urdf/combined.srdf",'r') as f:
			combined_srdf = f.read()
		t = tesseract.Tesseract()
		t.init(combined_urdf, combined_srdf, GazeboModelResourceLocator())
		self.t_env = t.getEnvironment()
		#update robot poses based on calibration file
		self.t_env.changeJointOrigin("ur_pose", H_UR)
		self.t_env.changeJointOrigin("sawyer_pose", H_Sawyer)
		self.t_env.changeJointOrigin("abb_pose", H_ABB)
		self.t_env.changeJointOrigin("staubli_pose", H_tx60)

		contact_distance=0.2
		monitored_link_names = self.t_env.getLinkNames()
		self.manager = self.t_env.getDiscreteContactManager()
		self.manager.setActiveCollisionObjects(monitored_link_names)
		self.manager.setContactDistanceThreshold(contact_distance)
		# viewer update
		self.viewer = TesseractViewer()
		self.viewer.update_environment(self.t_env, [0,0,0])
		self.viewer.start_serve_background()

		self.robot_def_dict={'ur':Robot(np.transpose(np.array(UR.robot_info.chains[0].H.tolist())),np.transpose(np.array(UR.robot_info.chains[0].P.tolist())),np.zeros(len(UR.robot_info.joint_info))),
		'sawyer':Robot(np.transpose(np.array(Sawyer.robot_info.chains[0].H.tolist())),np.transpose(np.array(Sawyer.robot_info.chains[0].P.tolist())),np.zeros(len(Sawyer.robot_info.joint_info))),
		'abb':Robot(np.transpose(np.array(ABB.robot_info.chains[0].H.tolist())),np.transpose(np.array(ABB.robot_info.chains[0].P.tolist())),np.zeros(len(ABB.robot_info.joint_info))),
		'staubli':Robot(np.transpose(np.array(tx60.robot_info.chains[0].H.tolist())),np.transpose(np.array(tx60.robot_info.chains[0].P.tolist())),np.zeros(len(tx60.robot_info.joint_info)))
		}


		#trajectories
		self.steps=300
		self.trajectory={'ur':np.zeros((self.steps,7)),'sawyer':np.zeros((self.steps,8)),'abb':np.zeros((self.steps,7)),'staubli':np.zeros((self.steps,7))}
		self.traj_joint_names={'ur':['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
		'sawyer':['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6'],
		'abb':['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
		'staubli':['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
		}
		self.time_step=0.02
		#initialize static trajectories
		for key, value in self.trajectory.items():
			for i in range(self.steps):
				try:
					value[i]=np.append([0],self.robot_state_list[self.dict[key]].InValue.joint_position)
				except:
					traceback.print_exc()
					value[i]=np.append([0],[0,0,0,0,0,0])
		self.inv={'ur':inv_ur,'sawyer':inv_sawyer,'abb':inv_abb,'staubli':inv_staubli}
		self.joint_names_traj={'ur':inv_ur,'sawyer':inv_sawyer,'abb':inv_abb,'staubli':inv_staubli}

		

		#register service constant
		self.JointTrajectoryWaypoint = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint")
		self.JointTrajectory = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectory")

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


	def distance_check_global(self,robot_name, joints_list):
		with self._lock:
			robot_idx=self.dict[robot_name]
			distance_report=RRN.GetStructureType("edu.rpi.robotics.distance.distance_report")
			distance_report1=distance_report()

			for i in range(self.num_robot):
				robot_joints=joints_list[i]
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
				distance_report1.J2C=J2C	
				
				
				return distance_report1

			return distance_report1


	def plan(self,robot_name,pd,Rd, obj_vel, capture_time):            #start and end configuration in joint space

		#update other robot static trajectories
		for key, value in self.trajectory.items():
			if value[0][0]==0:
				try:
					value[:]=np.append([0],self.robot_state_list[self.dict[key]].InValue.joint_position)
				except:
					value[:]=[0]*7


		Rd=Rd.reshape((3,3))
		plan_time=0.3
		start_time=time.time()+plan_time
		distance_threshold=0.1
		joint_threshold=0.1

		#get joint info in future 
		other_robot_trajectory_start_idx={'ur':self.steps-1,'sawyer':self.steps-1,'abb':self.steps-1,'staubli':self.steps-1}

		for key, value in self.trajectory.items():
			if key==robot_name:
				continue
			if value[0][0]!=0:
				other_robot_trajectory_start_idx[key] = (np.abs(value[:,0] - start_time)).argmin()

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


		while(np.linalg.norm(q_des[:-1]-q_cur[:-1])>joint_threshold):
			if step>self.steps:
				raise UnboundLocalError("Unplannable")
				return
			if np.linalg.norm(obj_vel)!=0:
				p_d=(pd+obj_vel*(time.time()-capture_time))

				q_des=self.inv[robot_name](p_d,Rd).reshape(n)
			else:
				p_d=pd
			

		#     get current H and J
			robot_pose=self.robot_state_list[self.dict[robot_name]].InValue.kin_chain_tcp[0]
			R_cur = q2R(np.array(robot_pose['orientation'].tolist()))

			p_cur=np.array(robot_pose['position'].tolist())

			J=robotjacobian(self.robot_def_dict[robot_name],q_cur)        #calculate current Jacobian
			Jp=J[3:,:]
			JR=J[:3,:]                              #decompose to position and orientation Jacobian

			ER=np.dot(R_cur,np.transpose(Rd))
			EP=p_cur-p_d                             #error in position and orientation
			#update future joint to distance checking

			joints_list=[self.trajectory['ur'][np.amin([step+other_robot_trajectory_start_idx['ur'],self.steps-1])][1:],self.trajectory['sawyer'][np.amin([step+other_robot_trajectory_start_idx['sawyer'],self.steps-1])][1:],
			self.trajectory['abb'][np.amin([step+other_robot_trajectory_start_idx['abb'],self.steps-1])][1:],self.trajectory['staubli'][np.amin([step+other_robot_trajectory_start_idx['staubli'],self.steps-1])][1:]]
			#update self joint position
			joints_list[self.dict[robot_name]]=q_cur
			distance_report=self.distance_check_global(robot_name,joints_list)


			Closest_Pt=distance_report.Closest_Pt
			Closest_Pt_env=distance_report.Closest_Pt_env
			dist=distance_report.min_distance
			J2C=distance_report.J2C

			if (Closest_Pt[0]!=0. and dist<distance_threshold):  

				print("qp triggering ",dist ) 
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
				
				b=np.array([-0.05])

				try:
					qdot=1.*normalize_dq(solve_qp(H, f,A,b))
					
				except:
					traceback.print_exc()

			else:
				if np.linalg.norm(q_des-q_cur)<0.5:
					qdot=normalize_dq(q_des-q_cur)
				else:
					qdot=2.*normalize_dq(q_des-q_cur)
			#update q_cur
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

		#estimate of time
		self.trajectory[robot_name][:,0]+=time.time()

		return traj

	def clear_traj(self,robot_name):
		#clear trajectory after execution
		self.trajectory[robot_name][:]=np.append([0],self.robot_state_list[self.dict[robot_name]].InValue.joint_position)

with RR.ServerNodeSetup("Distance_Service", 25522) as node_setup:
	cwd = os.getcwd()
	#register robot service definition
	directory='/home/iamnotedible/catkin_ws/src/robotraconteur_companion/robdef/group1/'
	os.chdir(directory)
	RRN.RegisterServiceTypesFromFiles(['com.robotraconteur.robotics.trajectory.robdef'],True)
	os.chdir(cwd)

	#register service file and service
	RRN.RegisterServiceTypeFromFile("../../robdef/edu.rpi.robotics.distance")
	distance_inst=create_impl()				#create obj
	# distance_inst.start()
	RRN.RegisterService("Environment","edu.rpi.robotics.distance.env",distance_inst)
	print("distance service started")

	# distance_inst.plan('abb',[0.5,0.5,0.5],np.array([1,0,0,0,1,0,0,0,1]),[0,0,0],0)
	input("Press enter to quit")
	distance_inst.stop()









