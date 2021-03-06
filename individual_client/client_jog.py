#Simple example Robot Raconteur Robot Jog Client
from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, yaml, argparse
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


#auto discovery
time.sleep(2)
res=RRN.FindServiceByType("com.robotraconteur.robotics.robot.Robot",
["rr+local","rr+tcp","rrs+tcp"])
url=None
for serviceinfo2 in res:
	if robot_name in serviceinfo2.NodeName:
		url=serviceinfo2.ConnectionURL
		break
if url==None:
	print('service not found')
	sys.exit()

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


# desired_joints=inv.inv(home)
desired_joints=np.zeros(num_joints)
desired_joints=[-np.pi/2,-1.2,0,0,0,0]
print(desired_joints)
# print(np.degrees(desired_joints))
robot.jog_freespace(desired_joints, 0.5*np.ones((num_joints,)), True)

# time.sleep(7)

print(inv.fwd(desired_joints))
robot.command_mode = halt_mode
