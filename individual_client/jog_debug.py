from RobotRaconteur.Client import *
import sys, time, yaml, traceback
import numpy as np
from importlib import import_module




robot_name='sawyer'


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
robot_height=robot_yaml['height']
home=robot_yaml['home']
obj_namelists=robot_yaml['obj_namelists']
pick_height=robot_yaml['pick_height']
place_height=robot_yaml['place_height']
robot_command=robot_yaml['robot_command']
tool_url=robot_yaml['tool_url']
gripper_orientation=robot_yaml['gripper_orientation']

robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)

def jog_joint_j(q):

	maxv=[1.]*(num_joints-1)+[4.5]
	robot.jog_freespace(q, np.array(maxv), True)

	return

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
mode = robot_const["RobotCommandMode"][robot_command]
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = jog_mode

num_joints=len(robot.robot_info.joint_info)
P=np.array(robot.robot_info.chains[0].P.tolist())
length=np.linalg.norm(P[1])+np.linalg.norm(P[2])+np.linalg.norm(P[3])
H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))


state_w = robot_sub.SubscribeWire("robot_state")
time.sleep(0.5)
robot_state_wire=state_w.TryGetInValue()

robot_state = robot_state_wire[1]
p=robot_state.kin_chain_tcp[0]['position'] 


R=R_ee.R_ee(0)
q=inv.inv(np.array([home[0],home[1]+0.1,home[2]-0.15]),R)
jog_joint_j(q)
q=inv.inv(np.array([home[0],home[1]+0.1,home[2]]),R)
jog_joint_j(q)



