#!/usr/bin/env python3
import tesseract
from tesseract_viewer import TesseractViewer
import os, re
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np
import yaml, time, traceback, threading, sys
import sys
sys.path.append('../')
from gazebo_model_resource_locator import GazeboModelResourceLocator


#link and joint names in urdf
Sawyer_joint_names=["right_j0","right_j1","right_j2","right_j3","right_j4","right_j5","right_j6"]
UR_joint_names=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
Sawyer_link_names=["right_l0","right_l1","right_l2","right_l3","right_l4","right_l5","right_l6","right_l1_2","right_l2_2","right_l4_2","right_hand"]
UR_link_names=['UR_base_link',"shoulder_link","upper_arm_link","forearm_link","wrist_1_link","wrist_2_link","wrist_3_link"]
ABB_joint_names=['ABB1200_joint_1','ABB1200_joint_2','ABB1200_joint_3','ABB1200_joint_4','ABB1200_joint_5','ABB1200_joint_6']
ABB_link_names=['ABB1200_base_link','ABB1200_link_1','ABB1200_link_2','ABB1200_link_3','ABB1200_link_4','ABB1200_link_5','ABB1200_link_6']


#convert 4x4 H matrix to 3x3 H matrix and inverse for mapping obj to robot frame
def H42H3(H):
	H3=np.linalg.inv(H[:2,:2])
	H3=np.hstack((H3,-np.dot(H3,np.array([[H[0][-1]],[H[1][-1]]]))))
	H3=np.vstack((H3,np.array([0,0,1])))
	return H3


#load calibration parameters
with open('../calibration/sawyer.yaml') as file:
	H_Sawyer 	= np.array(yaml.load(file)['H'],dtype=np.float64)
with open('../calibration/ur.yaml') as file:
	H_UR 		= np.array(yaml.load(file)['H'],dtype=np.float64)
with open('../calibration/abb.yaml') as file:
	H_ABB 	= np.array(yaml.load(file)['H'],dtype=np.float64)	

with open("../urdf/combined.urdf",'r') as f:
	combined_urdf = f.read()
with open("../urdf/combined.srdf",'r') as f:
	combined_srdf = f.read()

t = tesseract.Tesseract()
t.init(combined_urdf, combined_srdf, GazeboModelResourceLocator())
t_env = t.getEnvironment()
#update robot poses based on calibration file
t_env.changeJointOrigin("ur_pose", H_UR)
t_env.changeJointOrigin("sawyer_pose", H_Sawyer)
t_env.changeJointOrigin("abb_pose", H_ABB)

contact_distance=0.2
monitored_link_names = t_env.getLinkNames()
manager = t_env.getDiscreteContactManager()
manager.setActiveCollisionObjects(monitored_link_names)
manager.setContactDistanceThreshold(contact_distance)





t_env.setState(ABB_joint_names, [])

env_state = t_env.getCurrentState()
manager.setCollisionObjectsTransform(env_state.link_transforms)
contacts = manager.contactTest(2)

contact_vector = tesseract.flattenResults(contacts)

distances = np.array([c.distance for c in contact_vector])
nearest_points=np.array([c.nearest_points for c in contact_vector])
names = np.array([c.link_names for c in contact_vector])










