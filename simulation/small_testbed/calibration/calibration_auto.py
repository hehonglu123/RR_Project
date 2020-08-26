import numpy as np
import yaml
from RobotRaconteur.Client import *
import sys, time, copy


def calibrate(obj,ref):	#list of [x_c,y_c] and [x_r,y_r]
	obj=np.hstack((obj,np.ones((len(obj),1))))
	H,residuals,rank,s=np.linalg.lstsq(obj,ref,rcond=-1)
	H=np.vstack((np.transpose(H),[0,0,1]))
	#orthonormal
	u,s,vh=np.linalg.svd(H[:2,:2])
	H[:2,:2]=np.dot(u,vh)
	return H
#connect to cognex service to read robot eef pose
cognex_inst=RRN.ConnectService('rr+tcp://localhost:52222/?service=cognexsim')
#input robot name
robot_name=input("robot name: ")

R=np.zeros((3,3))
factor=1
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
elif robot_name=="staubli":
	robot=RRN.ConnectService('rr+tcp://localhost:58656?service=robot')
	key='staubli_eef'
	sys.path.append('../../../toolbox')
	from staubli_ik import inv
elif robot_name=="rp260":
	robot=RRN.ConnectService('rr+tcp://localhost:23333?service=robot')
	key='rp260_eef'
	sys.path.append('../../../toolbox')
	from rp260_ik import inv
	factor=0.05
	start=[0.05,-0.3,0.1]
	R=np.array([[1, 0,0],
			[0, -1, 0],
			[0,0,-1]])
else:
	print("no robot found")

	
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
print("moving to start point")
start_joints=inv(start,R).reshape((n,1))
robot.jog_joint(start_joints, np.ones((n,)), False, True)


#initialize coordinate list
robot_eef_coordinates=[[robot_state.InValue.kin_chain_tcp['position']['x'][0],robot_state.InValue.kin_chain_tcp['position']['y'][0]]]
cam_coordinates=[[detection_wire.InValue[key].x,detection_wire.InValue[key].y]]


###
now=time.time()
while time.time()-now<5:
	# joints=inv([(start[0]+np.sin(time.time()-now)*factor),(start[1]+(time.time()-now)*0.1),start[2]],R)
	waypoints=copy.deepcopy(start_joints)
	waypoints[0]+=(time.time()-now)/2.5
	robot.jog_joint(waypoints, np.ones((n,)), False, True)
	robot_eef_coordinates.append([robot_state.InValue.kin_chain_tcp['position']['x'][0],robot_state.InValue.kin_chain_tcp['position']['y'][0]])
	cam_coordinates.append([detection_wire.InValue[key].x,detection_wire.InValue[key].y])
	

print('num of samples: ', len(cam_coordinates))
H=calibrate(cam_coordinates, robot_eef_coordinates)

print(H)


