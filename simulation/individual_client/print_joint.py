#Simple example Robot Raconteur Sawyer Robot and Cognex client
#pick up and drop detected objects
from RobotRaconteur.Client import *
import numpy as np
import time
import traceback
####################Start Service and robot setup
# robot = RRN.ConnectService('rr+tcp://localhost:58653?service=robot')	#UR
# robot2= RRN.ConnectService('rr+tcp://localhost:58654?service=robot'  )	#Sawyer
# robot3= RRN.ConnectService('rr+tcp://localhost:58655?service=robot'  )	#ABB
# robot4= RRN.ConnectService('rr+tcp://localhost:58656?service=robot'  )	#Staubli
robot5= RRN.ConnectService('rr+tcp://localhost:23333?service=robot'  )	#Rp260


print(robot5.robot_info.device_info.device.name)
joints=robot5.robot_state.PeekInValue()[0].joint_position
pose=robot5.robot_state.PeekInValue()[0].kin_chain_tcp[0]['position'] 
print(joints)
print(pose)

# state_w = robot2.robot_state.Connect()
# time.sleep(1)
# robot_state = state_w.InValue
# print(robot_state.kin_chain_tcp)
# position=robot_state.kin_chain_tcp[0]['position'] 
# print(position)
# sawyer_joints=robot2.robot_state.PeekInValue()[0].joint_position
# sawyer_pose=robot2.robot_state.PeekInValue()[0].kin_chain_tcp[0]['position'] 
# print(sawyer_joints)
# print(sawyer_pose)