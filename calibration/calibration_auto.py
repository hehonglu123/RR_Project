import numpy as np
from RobotRaconteur.Client import *
import sys, time, yaml
from importlib import import_module
sys.path.append('../')
from vel_emulate_sub import EmulatedVelocityControl
from jog_joint import jog_joint
sys.path.append('../QP_planner')
from qp_calibration import plan
sys.path.append('../toolbox')
from general_robotics_toolbox import Robot, fwdkin


def calibrate(obj,ref):	#list of [x_c,y_c] and [x_r,y_r]
	obj=np.hstack((obj,np.ones((len(obj),1))))
	H,residuals,rank,s=np.linalg.lstsq(obj,ref,rcond=-1)
	H=np.vstack((np.transpose(H),[0,0,1]))
	#orthonormal
	u,s,vh=np.linalg.svd(H[:2,:2])
	H[:2,:2]=np.dot(u,vh)
	return H

key="eef"
R=[[1,0,0],[0,1,0],[0,0,1]]
#input robot name
#connection failed callback
def connect_failed(s, client_id, url, err):
    print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))

#read in robot name and import proper libraries
if (sys.version_info > (3, 0)):
	robot_name=input('robot name: ')
else:
	robot=raw_input('robot name: ')
sys.path.append('../toolbox')
inv = import_module(robot_name+'_ik')
R_ee = import_module('R_'+robot_name)

#########read in yaml file for robot client
with open(r'../client_yaml/client_'+robot_name+'.yaml') as file:
    robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
url=robot_yaml['url']
home=robot_yaml['home']
calibration_start=robot_yaml['calibration_start']
calibration_speed=robot_yaml['calibration_speed']


####################Start Service and robot setup
###########Connect to corresponding services, subscription mode
####subscription
cognex_sub=RRN.SubscribeService('rr+tcp://localhost:52222/?service=cognex')
robot_sub=RRN.SubscribeService(url)
####get client object
cognex_inst=cognex_sub.GetDefaultClientWait(1)
robot=robot_sub.GetDefaultClientWait(1)
####get subscription wire
#cognex detection wire
detection_wire=cognex_sub.SubscribeWire("detection_wire")
##robot wire
cmd_w = robot_sub.SubscribeWire("position_command")
state_w = robot_sub.SubscribeWire("robot_state")
####connection fail callback
cognex_sub.ClientConnectFailed += connect_failed
robot_sub.ClientConnectFailed += connect_failed

##########Initialize robot constants
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]	
position_mode = robot_const["RobotCommandMode"]["position_command"]
robot.command_mode = halt_mode

##########Initialize robot parameters	#need modify
num_joints=len(robot.robot_info.joint_info)
P=np.array(robot.robot_info.chains[0].P.tolist())
length=np.linalg.norm(P[1])+np.linalg.norm(P[2])+np.linalg.norm(P[3])
H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))

P[-2][0]-=0.045

robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))


#######move to start point
print("moving to start point")
robot.command_mode = position_mode 
vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w, 0.01)

jog_joint(robot,vel_ctrl,inv.inv(calibration_start,R),5)

#enable velocity mode
vel_ctrl.enable_velocity_mode()

#initialize coordinate list
joint_angles=[state_w.InValue.joint_position]
cam_coordinates=[[detection_wire.InValue[key].x,detection_wire.InValue[key].y]]

timestamp=None
now=time.time()
while time.time()-now<35:
	qdot=[calibration_speed]+[0]*(num_joints-1)
	vel_ctrl.set_velocity_command(np.array(qdot))

	cognex_wire=detection_wire.TryGetInValue()
	if cognex_wire[1][key].detected==True and cognex_wire[0] and cognex_wire[2]!=timestamp:
		timestamp=cognex_wire[2]
		joint_angles.append(state_w.InValue.joint_position)
		cam_coordinates.append([detection_wire.InValue[key].x,detection_wire.InValue[key].y])
	
vel_ctrl.set_velocity_command(np.zeros((num_joints,)))
vel_ctrl.disable_velocity_mode() 

#process data
eef=[]
num_samples=len(cam_coordinates)
print("num samples: ",num_samples)
for i in range(num_samples):
	transform=fwdkin(robot_def,joint_angles[i])
	p=transform.p
	eef.append(p.tolist()[:2])
H=calibrate(cam_coordinates, eef)
print(H)

with open(r'Sawyer.yaml', 'w') as file:
    documents = yaml.dump(H, file)

