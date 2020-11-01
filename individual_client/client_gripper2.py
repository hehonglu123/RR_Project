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


with open(r'../client_yaml/client_'+robot_name+'.yaml') as file:
    robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
tool_url=robot_yaml['tool_url']
####################Start Service and robot setup


gripper_sub=RRN.SubscribeService(tool_url)
gripper=gripper_sub.GetDefaultClientWait(1)
# gripper=RRN.ConnectService(tool_url)

gripper.close()
time.sleep(2)
gripper.open()
