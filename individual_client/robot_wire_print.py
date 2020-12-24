#Simple example Robot Raconteur Robot Print joint client
from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, yaml
from importlib import import_module


url='rr+tcp://localhost:58654?service=robot'

####################Start Service and robot setup

robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)

state_w = robot_sub.SubscribeWire("robot_state")


print(robot.robot_info.device_info.device.name)

while True:
	print(state_w.InValue().joint_position_command)

