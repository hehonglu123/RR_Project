#Simple example Robot Raconteur Robot Jog Client
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
from general_robotics_toolbox import Robot
sys.path.append('../QP_planner')
plan = import_module('plan_'+robot_name)
with open(r'../client_yaml/client_'+robot_name+'.yaml') as file:
    robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
url=robot_yaml['url']
home=robot_yaml['home']


####################Start Service and robot setup

robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)

num_joints=len(robot.robot_info.joint_info)
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = jog_mode


desired_joints=inv.inv(home)
# desired_joints=[-0.98001367,-0.56384668,0.17320605,1.84330664,2.18591016,1.25450391,-2.72375391]
print(np.degrees(desired_joints))
robot.jog_joint(desired_joints, np.ones((num_joints,)), True, True)



# robot.jog_joint(np.zeros(num_joints), np.ones((num_joints,)), False, True)

robot.command_mode = halt_mode
