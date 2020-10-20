#Simple example Robot Raconteur Robot Print joint client
from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, yaml
from importlib import import_module


#read in robot name and import proper libraries
if (sys.version_info > (3, 0)):
	robot_name=input('robot name: ')
else:
	robot=raw_input('robot name: ')
sys.path.append('../toolbox')
inv = import_module(robot_name+'_ik')
R_ee = import_module('R_'+robot_name)
from general_robotics_toolbox import Robot, q2R
sys.path.append('../QP_planner')
plan = import_module('plan_'+robot_name)
with open(r'../client_yaml/client_'+robot_name+'.yaml') as file:
    robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
url=robot_yaml['url']
home=robot_yaml['home']

####################Start Service and robot setup

robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)

state_w = robot_sub.SubscribeWire("robot_state")


print(robot.robot_info.device_info.device.name)

time.sleep(0.5)
robot_state_wire=state_w.TryGetInValue()
print("wire value set: ",robot_state_wire[0])
robot_state = robot_state_wire[1]
print("kin_chain_tcp: ", robot_state.kin_chain_tcp)
print("robot_joints: ", robot_state.joint_position)
position=robot_state.kin_chain_tcp[0]['position'] 
orientation=robot_state.kin_chain_tcp[0]['orientation'] 
print(position)
print(q2R(list(orientation)))

