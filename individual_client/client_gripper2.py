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
url=robot_yaml['url']
home=robot_yaml['home']

####################Start Service and robot setup

url="rr+tcp://[fe80::a2c:1efa:1c07:f043]:50500/?nodeid=07455191-1648-47ea-b1fe-d6555161badd&service=tool"
gripper_sub=RRN.SubscribeService(url)
gripper=gripper_sub.GetDefaultClientWait(1)

gripper.close()
time.sleep(2)
gripper.open()
