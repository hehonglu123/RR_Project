#!/usr/bin/env python3

#Robot Raconteur Project Robot Client
#pick up and drop detected objects
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np
from importlib import import_module
import time, traceback, sys, yaml, argparse

sys.path.append('../../')
from vel_emulate_sub import EmulatedVelocityControl


#connection failed callback
def connect_failed(s, client_id, url, err):
	print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))
#Accept the names of the webcams and the nodename from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--robot-name",type=str,help="List of camera names separated with commas")
args, _ = parser.parse_known_args()

robot_name=args.robot_name

sys.path.append('../../toolbox')
if robot_name=='ur':
	inv = import_module(robot_name+'_ik_sim')
else:
	inv = import_module(robot_name+'_ik')
R_ee = import_module('R_'+robot_name)
from general_robotics_toolbox import Robot

#########read in yaml file for robot client
with open(r'client_yaml/client_'+robot_name+'.yaml') as file:
	robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
url=robot_yaml['url']
home=robot_yaml['home']
obj_namelists=robot_yaml['obj_namelists']
pick_height=robot_yaml['pick_height']
place_height=robot_yaml['place_height']
joint_threshold=robot_yaml['joint_threshold']


####################Start Service and robot setup
###########Connect to corresponding services, subscription mode
####subscription
robot_sub=RRN.SubscribeService(url)
distance_sub=RRN.SubscribeService('rr+tcp://localhost:25522?service=Environment')
vacuum_sub=RRN.SubscribeService('rr+tcp://localhost:50000/?service=vacuumlink')
####get client object
robot=robot_sub.GetDefaultClientWait(1)
distance_inst=distance_sub.GetDefaultClientWait(1)
vacuum_inst=vacuum_sub.GetDefaultClientWait(1)

##robot wire
cmd_w = robot_sub.SubscribeWire("position_command")
state_w = robot_sub.SubscribeWire("robot_state")
####connection fail callback
robot_sub.ClientConnectFailed += connect_failed
distance_sub.ClientConnectFailed += connect_failed
vacuum_sub.ClientConnectFailed += connect_failed

##########Initialize robot constants
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]

position_mode = robot_const["RobotCommandMode"]["position_command"]
trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]
robot.command_mode = halt_mode

##########Connect to Cognex wire
# ##########Initialize velocity control parameters
RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",robot)
vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w, 0.01)
robot.command_mode = jog_mode 

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


#jog robot joint helper function
def jog_joint(q):
	robot.command_mode = halt_mode
	time.sleep(0.02)
	robot.command_mode = jog_mode
	maxv=[1.3]*(num_joints-1)+[3.2]
	robot.jog_freespace(q, np.array(maxv), True)
	robot.command_mode = halt_mode
	time.sleep(0.01)
	robot.command_mode = trajectory_mode
	return


def angle_threshold(angle):
	if (angle<-np.pi):
		angle+=2*np.pi
	elif (angle>np.pi):
		angle-=2*np.pi
	return angle

def conversion(x,y,height):
	p=np.dot(H_robot,np.array([[x],[y],[1]])).flatten()
	p[2]=height
	return p

def pick(obj):	

	#coordinate conversion
	p=conversion(obj.x,obj.y,pick_height)							
	#go there
	q=inv.inv(np.array([p[0],p[1],p[2]+0.05]))
	jog_joint(q)

	#move down
	q=inv.inv(np.array([p[0],p[1],p[2]]))
	jog_joint(q)

	#grab it
	print("get it")
	vacuum_inst.vacuum(robot_name,obj.name,1)
	q=inv.inv(np.array([p[0],p[1],p[2]+0.1]))
	jog_joint(q)
	return
def place(obj,slot):
	#get correct orientation
	angle=(slot.angle-obj.angle)

	R=R_ee.R_ee(angle_threshold(np.radians(angle)))

	p=conversion(slot.x,slot.y,place_height)
	#go there
	q=inv.inv(np.array([p[0],p[1],p[2]+0.05]),R)
	jog_joint(q)
	#move down
	q=inv.inv(np.array([p[0],p[1],p[2]]),R)
	jog_joint(q)

	

	print("dropped")
	vacuum_inst.vacuum(robot_name,obj.name,0)

	
	q=inv.inv(np.array([p[0],p[1],p[2]+0.05]))
	jog_joint(q)
	return

class objec(object):  # Add Node feature
    def __init__(self, x, y, angle,name):
        self.x = x
        self.y = y
        self.angle = angle
        self.name = name


from tree import solver,formD, train, execute
#1 for abb, 2 for sawyer
box1=[-0.6, 0.6]
box2=[0.6, 0.6]
des1=[-0.2,0]
des2=[0.2,0]
home1=np.array([-0.5,0,1.])
home2=np.array([0.5,0,1.])
objects=[]
desired_locations=[]
for i in range(4):
	objects.append(np.array([box1[0]-0.05,box1[1]-0.15+0.1*i,1.]))
	objects.append(np.array([box1[0]+0.05,box1[1]-0.15+0.1*i,1.]))
	desired_locations.append(np.array([des1[0]+0.1,des1[1]-0.5+0.3*i,1.]))
	desired_locations.append(np.array([des1[0]-0.1,des1[1]-0.5+0.3*i,1.]))


D,object_idx_list,box_idx_list,home_idx=formD(objects, desired_locations, home1)
# order=solver(D,object_idx_list,box_idx_list,home_idx)
Q=train(D,object_idx_list,box_idx_list,home_idx)
order=execute(Q,object_idx_list,box_idx_list,home_idx)

obj=objec(0,0,0,'round_bottle')
for index in order:
	if index==-1:
		######3go home
		q=inv.inv(home)
		jog_joint(q)
	elif index<len(objects):
		obj=objec(objects[index][0],objects[index][1],0,'round_bottle'+str(index))
		pick(obj)
	else:
		slot=objec(desired_locations[index-len(objects)][0],desired_locations[index-len(objects)][1],0,'noname')
		place(obj,slot)