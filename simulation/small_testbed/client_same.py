#!/usr/bin/env python

#Robot Raconteur Project Robot Client
#pick up and drop detected objects
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np
from importlib import import_module
import time, traceback, sys, yaml

sys.path.append('../../')
from vel_emulate_sub import EmulatedVelocityControl
from jog_joint import jog_joint

#connection failed callback
def connect_failed(s, client_id, url, err):
    print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))

#read in robot name and import proper libraries
if (sys.version_info > (3, 0)):
	robot_name=input('robot name: ')
else:
	robot=raw_input('robot name: ')
sys.path.append('../../toolbox')
inv = import_module(robot_name+'_ik')
R_ee = import_module('R_'+robot_name)
from general_robotics_toolbox import Robot
sys.path.append('QP_planner')
plan = import_module('plan_'+robot_name)
#########read in yaml file for robot client
with open(r'client_yaml/client_'+robot_name+'.yaml') as file:
    robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
url=robot_yaml['url']
height_offset=robot_yaml['height_offset']
home=robot_yaml['home']
obj_namelists=robot_yaml['obj_namelists']
pick_height=robot_yaml['pick_height']
place_height=robot_yaml['place_height']
destination=robot_yaml['destination']


####################Start Service and robot setup
###########Connect to corresponding services, subscription mode
####subscription
cognex_sub=RRN.SubscribeService('rr+tcp://localhost:52222/?service=cognexsim')
robot_sub=RRN.SubscribeService(url)
distance_sub=RRN.SubscribeService('rr+tcp://localhost:25522?service=Environment')
vacuum_sub=RRN.SubscribeService('rr+tcp://localhost:50000/?service=vacuumlink')
testbed_sub=RRN.SubscribeService('rr+tcp://localhost:6666?service=testbed')
####get client object
cognex_inst=cognex_sub.GetDefaultClientWait(1)
robot=robot_sub.GetDefaultClientWait(1)
distance_inst=distance_sub.GetDefaultClientWait(1)
vacuum_inst=vacuum_sub.GetDefaultClientWait(1)
testbed_inst=testbed_sub.GetDefaultClientWait(1)
####get subscription wire
##cognex detection wire
detection_wire=cognex_sub.SubscribeWire("detection_wire")
##robot wire
cmd_w = robot_sub.SubscribeWire("position_command")
state_w = robot_sub.SubscribeWire("robot_state")
####connection fail callback
cognex_sub.ClientConnectFailed += connect_failed
robot_sub.ClientConnectFailed += connect_failed
distance_sub.ClientConnectFailed += connect_failed
vacuum_sub.ClientConnectFailed += connect_failed
testbed_sub.ClientConnectFailed += connect_failed

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


##########load homogeneous transformation parameters Cognex->robot #need modify
slot_dict={'t_f':1,'p_f':0,'s_f':2,'b_f':3}	

transformations=distance_inst.transformations
H_robot=transformations[robot_name].H.reshape((transformations[robot_name].row,transformations[robot_name].column))

##########conveyor belt associated parameters
obj_vel=np.append(np.dot(H_robot[:-1,:-1],np.array([[0],[testbed_inst.speed]])).flatten(),0)


def single_move(p):
	plan.plan(robot,robot_def,p,R_ee.R_ee(0), vel_ctrl,distance_inst,robot_name,H_robot)
	return

def angle_threshold(angle):
	if (angle<-np.pi):
		angle+=2*np.pi
	elif (angle>np.pi):
		angle-=2*np.pi
	return angle

def conversion(x,y,height):
	p=np.dot(H_robot,np.array([[x],[y],[1]])).flatten()
	p[2]=height+height_offset
	return p

def pick(obj):	

	#coordinate conversion
	print("moving "+obj.name)
	p=conversion(obj.x,obj.y,pick_height)							
	R=R_ee.R_ee(angle_threshold(np.radians(obj.angle)))
	#move to object above
	plan.plan(robot,robot_def,[p[0],p[1],p[2]+0.1],R,vel_ctrl,distance_inst,robot_name,H_robot)
	#move down
	q=inv.inv(np.array([p[0],p[1],p[2]]),R)
	jog_joint(robot,vel_ctrl,q,.5)

	#grab it
	print("get it")
	time.sleep(1)
	vacuum_inst.vacuum(robot_name,obj.name,1)
	q=inv.inv(np.array([p[0],p[1],p[2]+0.1]),R)
	jog_joint(robot,vel_ctrl,q,.5)
	return
def place(obj,destination):

	#coordinate conversion

	p=conversion(destination[0],destination[1],place_height+testbed_inst.num_blocks*0.025)

	#get correct orientation
	angle=0

	R=R_ee.R_ee(angle_threshold(np.radians(angle)))
	
	plan.plan(robot,robot_def,[p[0],p[1],p[2]+0.1],R,vel_ctrl,distance_inst,robot_name,H_robot)


	#move down through jog joint
	q=inv.inv(p,R)


	jog_joint(robot,vel_ctrl,q,.5)
	time.sleep(0.02)

	time.sleep(1)
	print("dropped")
	vacuum_inst.vacuum(robot_name,obj.name,0)
	
	q=inv.inv([p[0],p[1],p[2]+0.1],R)
	jog_joint(robot,vel_ctrl,q,.5)
	return


#wait until wire value set
while True:
	if detection_wire.TryGetInValue()[0]:
		break

obj_grabbed=None

while True:
	single_move(home)
	if vacuum_inst.actions[robot_name]!=1:
		
		for i in range(len(obj_namelists)):
			#check current robot free, and pick the object
			obj=detection_wire.InValue[obj_namelists[i]]
			if obj.detected and testbed_inst.speed==0:
				try:
					pick(obj)
					obj_grabbed=obj
					testbed_inst.onboard=[]
					break
				except ValueError:
					pass
				except UnboundLocalError:
					pass	

	if vacuum_inst.actions[robot_name]==1:
		try:
			place(obj_grabbed,destination)
			
		except ValueError:
			pass
		except UnboundLocalError:
			pass
		except:
			traceback.print_exc()

	


