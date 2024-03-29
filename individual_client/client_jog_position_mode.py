#Simple example Robot Raconteur Robot Jog Client
from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, yaml
from importlib import import_module
sys.path.append('../')
from vel_emulate_sub import EmulatedVelocityControl
from jog_joint import jog_joint

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
robot.command_mode = position_mode

##robot wire
cmd_w = robot_sub.SubscribeWire("position_command")
state_w = robot_sub.SubscribeWire("robot_state")


vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w, 0.01)

desired_joints=inv.inv(home)
# desired_joints=np.zeros(num_joints)
print(np.degrees(desired_joints))
jog_joint(robot,vel_ctrl,desired_joints,3)
# for i in range(100):
# 	vel_ctrl.set_joint_command_position(desired_joints)
# 	time.sleep(0.02)


robot.command_mode = halt_mode
