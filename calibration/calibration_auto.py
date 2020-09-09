import numpy as np
from RobotRaconteur.Client import *
import sys, time, yaml
from importlib import import_module
sys.path.append('../')
from vel_emulate_sub import EmulatedVelocityControl


def calibrate(obj,ref):	#list of [x_c,y_c] and [x_r,y_r]
	obj=np.hstack((obj,np.ones((len(obj),1))))
	H,residuals,rank,s=np.linalg.lstsq(obj,ref,rcond=-1)
	H=np.vstack((np.transpose(H),[0,0,1]))
	#orthonormal
	u,s,vh=np.linalg.svd(H[:2,:2])
	H[:2,:2]=np.dot(u,vh)
	return H

key="robot_eef"
R=[[1,0,0],[0,1,0],[0,0,1]]
# R=[[0,0,1],[-1,0,0],[0,-1,0]]
#input robot name
#connection failed callback
def connect_failed(s, client_id, url, err):
    print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))

#read in robot name and import proper libraries
if (sys.version_info > (3, 0)):
	robot_name=input('robot name: ')
else:
	robot=raw_input('robot name: ')
sys.path.append('../toolbox')
inv = import_module(robot_name+'_ik')
R_ee = import_module('R_'+robot_name)
from general_robotics_toolbox import Robot
sys.path.append('../QP_planner')
plan = import_module('plan_'+robot_name)
#########read in yaml file for robot client
with open(r'../client_yaml/client_'+robot_name+'.yaml') as file:
    robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
url=robot_yaml['url']
height_offset=robot_yaml['height_offset']
home=robot_yaml['home']
obj_namelists=robot_yaml['obj_namelists']
pick_height=robot_yaml['pick_height']
place_height=robot_yaml['place_height']


####################Start Service and robot setup
###########Connect to corresponding services, subscription mode
####subscription
# cognex_sub=RRN.SubscribeService('rr+tcp://localhost:52222/?service=cognexsim')
robot_sub=RRN.SubscribeService(url)
# distance_sub=RRN.SubscribeService('rr+tcp://localhost:25522?service=Environment')
####get client object
# cognex_inst=cognex_sub.GetDefaultClientWait(1)
robot=robot_sub.GetDefaultClientWait(1)
# distance_inst=distance_sub.GetDefaultClientWait(1)
####get subscription wire
##cognex detection wire
# detection_wire=cognex_sub.SubscribeWire("detection_wire")
##robot wire
cmd_w = robot_sub.SubscribeWire("position_command")
state_w = robot_sub.SubscribeWire("robot_state")
####connection fail callback
# cognex_sub.ClientConnectFailed += connect_failed
robot_sub.ClientConnectFailed += connect_failed
# distance_sub.ClientConnectFailed += connect_failed

##########Initialize robot constants
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]	
position_mode = robot_const["RobotCommandMode"]["position_command"]
robot.command_mode = halt_mode

##########Connect to Cognex wire
# ##########Initialize velocity control parameters
# RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",robot)
# vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w, 0.01)
robot.command_mode = jog_mode 


##########Initialize robot parameters	#need modify
num_joints=len(robot.robot_info.joint_info)
P=np.array(robot.robot_info.chains[0].P.tolist())
length=np.linalg.norm(P[1])+np.linalg.norm(P[2])+np.linalg.norm(P[3])
H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
# joint_type = robot.robot_info.joint_info.joint_type.tolist()
robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))

# transformations=distance_inst.transformations
# H_robot=transformations[robot_name].H.reshape((transformations[robot_name].row,transformations[robot_name].column))

#######move to start point
robot.jog_joint(inv.inv([0.5,-0.3,0.2],R).reshape((num_joints,1)), np.ones((num_joints,)), False, True)
print("moving to start point")

#initialize coordinate list
# robot_eef_coordinates=[[robot_state.InValue.kin_chain_tcp['position']['x'][0],robot_state.InValue.kin_chain_tcp['position']['y'][0]]]
# cam_coordinates=[[detection_wire.InValue[key].x,detection_wire.InValue[key].y]]


###
# now=time.time()
# while time.time()-now<5:
# 	plan.plan(robot,robot_def,[0.5,-0.3+(time.time()-now)/8,0.2],R, vel_ctrl,distance_inst,robot_name,H_robot)
	# robot_eef_coordinates.append([robot_state.InValue.kin_chain_tcp['position']['x'][0],robot_state.InValue.kin_chain_tcp['position']['y'][0]])
	# cam_coordinates.append([detection_wire.InValue[key].x,detection_wire.InValue[key].y])
	

# print(len(cam_coordinates))
# H=calibrate(cam_coordinates, robot_eef_coordinates)

# print(H)

