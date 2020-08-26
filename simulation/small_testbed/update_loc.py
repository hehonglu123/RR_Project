#!/usr/bin/env python

#update robot pose based on calibration file in gazebo
import yaml
import numpy as np
import traceback
from RobotRaconteur.Client import *

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',Attach)
attach_srv.wait_for_service()
req = AttachRequest()

import sys
sys.path.append('../toolbox/')
from general_robotics_toolbox import R2q	#convert R to quaternion


def decomp(H):
	return R2q(H[:3,:3]),H[:3,3]

server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
w=server.get_worlds(str(server.world_names[0]))
pose_dtype = RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose", server)

with open('calibration/UR2.yaml') as file:
	H_UR 		= np.array(yaml.load(file)['H'],dtype=np.float64)
with open('calibration/Sawyer2.yaml') as file:
	H_Sawyer 	= np.array(yaml.load(file)['H'],dtype=np.float64)
with open('calibration/ABB2.yaml') as file:
	H_ABB 	= np.array(yaml.load(file)['H'],dtype=np.float64)
with open('calibration/tx60.yaml') as file:
	H_tx60 	= np.array(yaml.load(file)['H'],dtype=np.float64)

q_UR,d_UR=decomp(H_UR)
q_Sawyer,d_Sawyer=decomp(H_Sawyer)
q_ABB,d_ABB=decomp(H_ABB)
q_tx60,d_tx60=decomp(H_tx60)

try:
	model_pose = np.zeros((1,), dtype = pose_dtype)
	model_pose["orientation"]['w'] = q_UR[0]
	model_pose["orientation"]['x'] = q_UR[1]
	model_pose["orientation"]['y'] = q_UR[2]
	model_pose["orientation"]['z'] = q_UR[3]
	model_pose["position"]['x']=d_UR[0]
	model_pose["position"]['y']=d_UR[1]
	model_pose["position"]['z']=d_UR[2]
	UR=w.get_models('ur5')
	UR.setf_world_pose(model_pose)

	req.model_name_1 = "ur5"
	req.link_name_1 = "base_link"
	req.model_name_2 = "ground_plane"
	req.link_name_2 = "link"
	attach_srv.call(req)



	model_pose = np.zeros((1,), dtype = pose_dtype)
	model_pose["orientation"]['w'] = q_Sawyer[0]
	model_pose["orientation"]['x'] = q_Sawyer[1]
	model_pose["orientation"]['y'] = q_Sawyer[2]
	model_pose["orientation"]['z'] = q_Sawyer[3]
	model_pose["position"]['x']=d_Sawyer[0]
	model_pose["position"]['y']=d_Sawyer[1]
	model_pose["position"]['z']=d_Sawyer[2]
	Sawyer=w.get_models('sawyer')
	Sawyer.setf_world_pose(model_pose)

	req.model_name_1 = "sawyer"
	req.link_name_1 = "base"
	req.model_name_2 = "ground_plane"
	req.link_name_2 = "link"
	attach_srv.call(req)

	model_pose = np.zeros((1,), dtype = pose_dtype)
	model_pose["orientation"]['w'] = q_ABB[0]
	model_pose["orientation"]['x'] = q_ABB[1]
	model_pose["orientation"]['y'] = q_ABB[2]
	model_pose["orientation"]['z'] = q_ABB[3]
	model_pose["position"]['x']=d_ABB[0]
	model_pose["position"]['y']=d_ABB[1]
	model_pose["position"]['z']=d_ABB[2]
	ABB=w.get_models('ABB1200')
	ABB.setf_world_pose(model_pose)

	req.model_name_1 = "ABB1200"
	req.link_name_1 = "base_link"
	req.model_name_2 = "ground_plane"
	req.link_name_2 = "link"
	attach_srv.call(req)

	model_pose = np.zeros((1,), dtype = pose_dtype)
	model_pose["orientation"]['w'] = q_tx60[0]
	model_pose["orientation"]['x'] = q_tx60[1]
	model_pose["orientation"]['y'] = q_tx60[2]
	model_pose["orientation"]['z'] = q_tx60[3]
	model_pose["position"]['x']=d_tx60[0]
	model_pose["position"]['y']=d_tx60[1]
	model_pose["position"]['z']=d_tx60[2]
	tx60=w.get_models('tx60')
	tx60.setf_world_pose(model_pose)

	req.model_name_1 = "tx60"
	req.link_name_1 = "base_link"
	req.model_name_2 = "ground_plane"
	req.link_name_2 = "link"
	attach_srv.call(req)
except:
	traceback.print_exc()