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
# cognex_inst=RRN.ConnectService('rr+tcp://localhost:52222/?service=cognexsim')
distance_inst=RRN.ConnectService('rr+tcp://localhost:25522?service=Environment')
key="robot_eef"
#input robot name
if (sys.version_info > (3, 0)):
	robot_name=input('robot name: ')
else:
	robot=raw_input('robot name: ')
R=[[1,0,0],[0,1,0],[0,0,1]]
#connect to robot service, import correct inv lib based on robot name
if robot_name=="ur":
	robot=RRN.ConnectService('rr+tcp://localhost:58653?service=robot')
	sys.path.append('../toolbox')
	from ur_ik import inv

elif robot_name=="sawyer":
	robot = RRN.ConnectService('rr+tcp://localhost:58654?service=sawyer')
	sys.path.append('../toolbox')
	from sawyer_ik import inv
	R=[[1,0,0],[0,1,0],[0,0,1]]
elif robot_name=="abb":
	robot=RRN.ConnectService('rr+tcp://localhost:58655?service=robot')
	sys.path.append('../toolbox')
	from abb_ik import inv
else:
	robot=RRN.ConnectService('rr+tcp://localhost:58656?service=robot')
	sys.path.append('../toolbox')
	from staubli_ik import inv
	
#connect to wires
# detection_wire=cognex_inst.detection_wire.Connect()
# robot_state = robot.robot_state.Connect()

##########Initialize robot constants
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
robot.command_mode = halt_mode

##########Connect to Cognex wire
# ##########Initialize velocity control parameters
RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",robot)
vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w, 0.01)
robot.command_mode = position_mode 

##########Initialize robot parameters	#need modify
num_joints=len(robot.robot_info.joint_info)
P=np.array(robot.robot_info.chains[0].P.tolist())
length=np.linalg.norm(P[1])+np.linalg.norm(P[2])+np.linalg.norm(P[3])
H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
# joint_type = robot.robot_info.joint_info.joint_type.tolist()
robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))

transformations=distance_inst.transformations
H_robot=transformations[robot_name].H.reshape((transformations[robot_name].row,transformations[robot_name].column))

#######move to start point
robot.jog_joint(inv([0.5,-0.3,0.2],R).reshape((n,1)), np.ones((n,)), False, True)
print("moving to start point")

#initialize coordinate list
# robot_eef_coordinates=[[robot_state.InValue.kin_chain_tcp['position']['x'][0],robot_state.InValue.kin_chain_tcp['position']['y'][0]]]
# cam_coordinates=[[detection_wire.InValue[key].x,detection_wire.InValue[key].y]]


###
now=time.time()
while time.time()-now<5:
	plan.plan(robot,robot_def,[0.5,-0.3+(time.time()-now)/8,0.2],R, vel_ctrl,distance_inst,robot_name,H_robot)
	# robot_eef_coordinates.append([robot_state.InValue.kin_chain_tcp['position']['x'][0],robot_state.InValue.kin_chain_tcp['position']['y'][0]])
	# cam_coordinates.append([detection_wire.InValue[key].x,detection_wire.InValue[key].y])
	

# print(len(cam_coordinates))
# H=calibrate(cam_coordinates, robot_eef_coordinates)

# print(H)


