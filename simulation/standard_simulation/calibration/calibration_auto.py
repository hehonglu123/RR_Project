import numpy as np
import yaml
from RobotRaconteur.Client import *
import sys, time


def calibrate(obj,ref):	#list of [x_c,y_c] and [x_r,y_r]
	obj=np.hstack((obj,np.ones((len(obj),1))))
	H,residuals,rank,s=np.linalg.lstsq(obj,ref,rcond=-1)
	H=np.vstack((np.transpose(H),[0,0,1]))
	#orthonormal
	u,s,vh=np.linalg.svd(H[:2,:2])
	H[:2,:2]=np.dot(u,vh)
	return H
#connect to cognex service to read robot eef pose
cognex_inst=RRN.ConnectService('rr+tcp://localhost:52222/?service=cognex')
#read in robot name and import proper libraries
if (sys.version_info > (3, 0)):
	robot_name=input('robot name: ')
else:
	robot=raw_input('robot name: ')

#connect to robot service, import correct inv lib based on robot name
if robot_name=="ur":
	robot=RRN.ConnectService('rr+tcp://localhost:58653?service=robot')
	key='ur_eef'
	sys.path.append('../../../toolbox')
	from ur_ik import inv

elif robot_name=="sawyer":
	robot = RRN.ConnectService('rr+tcp://localhost:58654?service=robot')
	key='sawyer_eef'
	sys.path.append('../../../toolbox')
	from sawyer_ik import inv
elif robot_name=="abb":
	robot=RRN.ConnectService('rr+tcp://localhost:58655?service=robot')
	key='abb_eef'
	sys.path.append('../../../toolbox')
	from abb_ik import inv
else:
	robot=RRN.ConnectService('rr+tcp://localhost:58656?service=robot')
	key='staubli_eef'
	sys.path.append('../../../toolbox')
	from staubli_ik import inv
	
#connect to wires
detection_wire=cognex_inst.detection_wire.Connect()
robot_state = robot.robot_state.Connect()

##############robot param setup
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = jog_mode
n= len(robot.robot_info.joint_info)

#######move to start point
robot.jog_joint(inv([0.35,-0.3,0.3]).reshape((n,1)), np.ones((n,)), False, True)
print("moving to start point")

#initialize coordinate list
robot_eef_coordinates=[[robot_state.InValue.kin_chain_tcp['position']['x'][0],robot_state.InValue.kin_chain_tcp['position']['y'][0]]]
cam_coordinates=[[detection_wire.InValue[key].x,detection_wire.InValue[key].y]]


###
now=time.time()
while time.time()-now<5:
	joints=inv([0.25+np.sin(time.time()-now)/8.,-0.3+(time.time()-now)/8,0.3])
	robot.jog_joint(joints.reshape((n,1)), np.ones((n,)), False, True)
	robot_eef_coordinates.append([robot_state.InValue.kin_chain_tcp['position']['x'][0],robot_state.InValue.kin_chain_tcp['position']['y'][0]])
	cam_coordinates.append([detection_wire.InValue[key].x,detection_wire.InValue[key].y])
	

print(len(cam_coordinates))
H=calibrate(cam_coordinates, robot_eef_coordinates)

print(H)


