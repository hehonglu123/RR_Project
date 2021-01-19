#Simple example Robot Raconteur robot rolling client
from RobotRaconteur.Client import *
import numpy as np
####################Start Service and robot setup
url='rr+tcp://localhost:25522?service=Environment'
robot_height_dict={'ur':1.0,'sawyer':0.93,'abb':0.85,'staubli':0.85}

robot_name=raw_input('robot name: ')
x=float(raw_input('desired x: '))
y=float(raw_input('desired y: '))
angle=float(raw_input('desired angle (degrees): '))

obj = RRN.ConnectService(url)
obj.roll(robot_name,x,y,robot_height_dict[robot_name],np.radians(angle))
