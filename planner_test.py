#!/usr/bin/env python3

#Robot Raconteur Project Robot Client
#pick up and drop detected objects
from RobotRaconteur.Client import *

import numpy as np
from importlib import import_module
import time, traceback, sys, yaml, argparse, copy

#connection failed callback
def connect_failed(s, client_id, url, err):
	print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))

#Accept the names of the webcams and the nodename from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--robot-name",type=str,help="List of camera names separated with commas")
args, _ = parser.parse_known_args()

robot_name=args.robot_name

#read conveyor info
with open(r'client_yaml/testbed.yaml') as file:
	testbed_yaml = yaml.load(file, Loader=yaml.FullLoader)
conveyor_speed=testbed_yaml['conveyor_speed']


sys.path.append('toolbox')
inv = import_module(robot_name+'_ik')
R_ee = import_module('R_'+robot_name)
from general_robotics_toolbox import Robot
#old gripper
# sys.path.append('gripper_func')
# gripper = import_module(robot_name+'_gripper')

#########read in yaml file for robot client
with open(r'client_yaml/client_'+robot_name+'.yaml') as file:
	robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
url=robot_yaml['url']
robot_height=robot_yaml['height']
home=robot_yaml['home']
obj_namelists=robot_yaml['obj_namelists']
pick_height=robot_yaml['pick_height']
place_height=robot_yaml['place_height']
robot_command=robot_yaml['robot_command']
tool_url=robot_yaml['tool_url']
gripper_orientation=robot_yaml['gripper_orientation']
tool_length=robot_yaml['tool_length']
joint_threshold=robot_yaml['joint_threshold']
speed=robot_yaml['speed']
####################Start Service and robot setup
###########Connect to corresponding services, subscription mode
####subscription
cognex_sub=RRN.SubscribeService('rr+tcp://[fe80::922f:c9e6:5fe5:51d1]:52222/?nodeid=87518815-d3a3-4e33-a1be-13325da2461f&service=cognex')
detection_sub=RRN.SubscribeService('rr+tcp://localhost:52222/?service=detection')
robot_sub=RRN.SubscribeService(url)
tool_sub=RRN.SubscribeService(tool_url)
distance_sub=RRN.SubscribeService('rr+tcp://localhost:25522?service=Environment')
####get client object
robot=robot_sub.GetDefaultClientWait(1)
distance_inst=distance_sub.GetDefaultClientWait(1)
#new gripper
gripper=tool_sub.GetDefaultClientWait(1)


gripper_on=False
####get subscription wire
##cognex detection wire
detection_wire=cognex_sub.SubscribeWire("detection_wire")
detection_wire_kinect=detection_sub.SubscribeWire("detection_wire")
##distance report wire
distance_report_wire=distance_sub.SubscribeWire("distance_report_wire")

##robot wire
pos_w=robot_sub.SubscribeWire('position_command')
vel_w=robot_sub.SubscribeWire('velocity_command')
if robot_command=='position_command':
	cmd_w = pos_w
else:
	cmd_w = vel_w
state_w = robot_sub.SubscribeWire("robot_state")
####connection fail callback
cognex_sub.ClientConnectFailed += connect_failed
robot_sub.ClientConnectFailed += connect_failed
distance_sub.ClientConnectFailed += connect_failed

##########Initialize robot constants
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
JointTrajectoryWaypoint = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint",robot)
JointTrajectory = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectory",robot)
joint_names = [j.joint_identifier.name for j in robot.robot_info.joint_info]
RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",robot)

halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
velocity_mode = robot_const["RobotCommandMode"]["velocity_command"]
trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]

robot.command_mode = halt_mode

# ##########Initialize robot velocity control mode
if robot_command=="position_command":
	from vel_emulate_sub import EmulatedVelocityControl
	vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w)
else:
	from vel_ctrl_sub import VelocityControl
	vel_ctrl = VelocityControl(robot,state_w, cmd_w)
robot.command_mode = trajectory_mode 

##########Initialize robot parameters
num_joints=len(robot.robot_info.joint_info)
P=np.array(robot.robot_info.chains[0].P.tolist())
length=np.linalg.norm(P[1])+np.linalg.norm(P[2])+np.linalg.norm(P[3])
P[-1]+=np.array(tool_length)
H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))


##########load homogeneous transformation parameters Cognex->robot 
transformations=distance_inst.transformations
H_robot=transformations[robot_name].H.reshape((transformations[robot_name].row,transformations[robot_name].column))

##########conveyor belt associated parameters
obj_vel=np.append(np.dot(H_robot[:-1,:-1],np.array([[conveyor_speed],[0]])).flatten(),0)















traj=distance_inst.plan(robot_name,speed,[-0.3,-0.3,0.2],list(R_ee.R_ee(0).flatten()),joint_threshold,[0.,0.,0.],0)