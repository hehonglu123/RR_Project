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
height_offset=robot_yaml['height_offset']
home=robot_yaml['home']
obj_namelists=robot_yaml['obj_namelists']
pick_height=robot_yaml['pick_height']
place_height=robot_yaml['place_height']

####################Start Service and robot setup

robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)


robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = jog_mode


desired_joints=inv.inv(home)
# robot.jog_joint(desired_joints, np.ones((7,)), True, True)


robot.jog_joint([ 0.96428242, -1.68345277, -0.29011671,  2.43833588, -0.04434084,  0.81173059, -2.43520848], np.ones((7,)), True, True)
# time.sleep(5)
# robot.jog_joint(np.zeros(7), np.ones((7,)), False, True)

robot.command_mode = halt_mode