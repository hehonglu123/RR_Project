from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, yaml

#read in robot name and import proper libraries
if (sys.version_info > (3, 0)):
	robot_name=input('robot name: ')
else:
	robot=raw_input('robot name: ')
with open(r'../client_yaml/client_'+robot_name+'.yaml') as file:
    robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
url=robot_yaml['url']

robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)

robot.reset_errors()
# robot.enable()
