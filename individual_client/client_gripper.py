#Simple example Robot Raconteur Sawyer Robot and Cognex client
#pick up and drop detected objects
from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, yaml
from importlib import import_module


#read in robot name and import proper libraries
if (sys.version_info > (3, 0)):
	robot_name=input('robot name: ')
else:
	robot=raw_input('robot name: ')

sys.path.append('../gripper_func')
gripper = import_module(robot_name+'_gripper')

with open(r'../client_yaml/client_'+robot_name+'.yaml') as file:
    robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
url=robot_yaml['url']
home=robot_yaml['home']

####################Start Service and robot setup

robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)

# gripper.initialize(robot)
# time.sleep(2)

gripper.gripper(robot,0)
time.sleep(2)
gripper.gripper(robot,1)
