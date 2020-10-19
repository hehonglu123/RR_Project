#!/usr/bin/env python3

#Robot Raconteur Project Robot Client
#pick up and drop detected objects
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np
from importlib import import_module
import time, traceback, sys, yaml, argparse
from jog_joint import jog_joint as jg2

#connection failed callback
def connect_failed(s, client_id, url, err):
	print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))

#Accept the names of the webcams and the nodename from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--robot-name",type=str,help="List of camera names separated with commas")
args, _ = parser.parse_known_args()

robot_name=args.robot_name

#read conveyor info
with open(r'client_yaml/testbed.yaml') as file:
	testbed_yaml = yaml.load(file, Loader=yaml.FullLoader)
conveyor_speed=testbed_yaml['conveyor_speed']


sys.path.append('toolbox')
inv = import_module(robot_name+'_ik')
R_ee = import_module('R_'+robot_name)
from general_robotics_toolbox import Robot
sys.path.append('QP_planner')
plan = import_module('plan_'+robot_name)
#old gripper
sys.path.append('gripper_func')
gripper = import_module(robot_name+'_gripper')

#########read in yaml file for robot client
with open(r'client_yaml/client_'+robot_name+'.yaml') as file:
	robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
url=robot_yaml['url']
robot_height=robot_yaml['height']
home=robot_yaml['home']
obj_namelists=robot_yaml['obj_namelists']
pick_height=robot_yaml['pick_height']
place_height=robot_yaml['place_height']
robot_command=robot_yaml['robot_command']
tool_url=robot_yaml['tool_url']
gripper_orientation=robot_yaml['gripper_orientation']


####################Start Service and robot setup
###########Connect to corresponding services, subscription mode
####subscription
cognex_sub=RRN.SubscribeService('rr+tcp://[fe80::922f:c9e6:5fe5:51d1]:52222/?nodeid=87518815-d3a3-4e33-a1be-13325da2461f&service=cognex')
robot_sub=RRN.SubscribeService(url)
# tool_sub=RRN.SubscribeService(tool_url)
distance_sub=RRN.SubscribeService('rr+tcp://localhost:25522?service=Environment')
####get client object
cognex_inst=cognex_sub.GetDefaultClientWait(1)
robot=robot_sub.GetDefaultClientWait(1)
distance_inst=distance_sub.GetDefaultClientWait(1)
#new gripper
# gripper=tool_sub.GetDefaultClientWait(1)
gripper_on=False
####get subscription wire
##cognex detection wire
detection_wire=cognex_sub.SubscribeWire("detection_wire")
##robot wire
cmd_w = robot_sub.SubscribeWire(robot_command)
state_w = robot_sub.SubscribeWire("robot_state")
####connection fail callback
cognex_sub.ClientConnectFailed += connect_failed
robot_sub.ClientConnectFailed += connect_failed
distance_sub.ClientConnectFailed += connect_failed

##########Initialize robot constants
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
mode = robot_const["RobotCommandMode"][robot_command]
robot.command_mode = halt_mode

# ##########Initialize robot velocity control mode
if robot_command=="position_command":
	from vel_emulate_sub import EmulatedVelocityControl
	vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w)
else:
	from vel_ctrl_sub import VelocityControl
	vel_ctrl = VelocityControl(robot,state_w, cmd_w)
robot.command_mode = mode 

##########Initialize robot parameters
num_joints=len(robot.robot_info.joint_info)
P=np.array(robot.robot_info.chains[0].P.tolist())
length=np.linalg.norm(P[1])+np.linalg.norm(P[2])+np.linalg.norm(P[3])
H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))


##########load homogeneous transformation parameters Cognex->robot 
transformations=distance_inst.transformations
H_robot=transformations[robot_name].H.reshape((transformations[robot_name].row,transformations[robot_name].column))

##########conveyor belt associated parameters
obj_vel=np.append(np.dot(H_robot[:-1,:-1],np.array([[conveyor_speed],[0]])).flatten(),0)

# #jog robot joint helper function
# def jog_joint(q,qdot=0.5*np.ones((num_joints,))):
# 	robot.command_mode = halt_mode
# 	time.sleep(0.01)
# 	robot.command_mode = jog_mode
# 	robot.jog_freespace(q, qdot, True)
# 	robot.command_mode = halt_mode
# 	time.sleep(0.01)
# 	robot.command_mode = mode
# 	return
def jog_joint(q,t=1):
	#parameter setup
	n= len(robot.robot_info.joint_info)
	#enable velocity mode
	vel_ctrl.enable_velocity_mode()
	
	qdot=1.05*(q-vel_ctrl.joint_position())/t
	now=time.time()
	while np.linalg.norm(q-vel_ctrl.joint_position())>0.01 and time.time()-now<t:
		vel_ctrl.set_velocity_command(qdot)

	vel_ctrl.set_velocity_command(np.zeros((n,)))
	vel_ctrl.disable_velocity_mode() 


# #move to a point with planner
def single_move(p):
	plan.plan(robot,robot_def,p,R_ee.R_ee(0), vel_ctrl,distance_inst,robot_name,H_robot)
	return
#threshold angle
def angle_threshold(angle):
	if (angle<-np.pi):
		angle+=2*np.pi
	elif (angle>np.pi):
		angle-=2*np.pi
	return angle
#convert cognex frame to robot frame
def conversion(x,y,height):
	p=np.dot(H_robot,np.array([[x],[y],[1]])).flatten()
	p[2]=height
	return p

def pick(obj):	
	global gripper_on

	#coordinate conversion
	print("picking "+obj.name)
	obj_pick_height=pick_height+testbed_yaml[obj.name]
	p=conversion(obj.x,obj.y,obj_pick_height)							
	print(p)
	R=R_ee.R_ee(angle_threshold(np.radians(obj.angle)-gripper_orientation))
	#move to object above
	plan.plan(robot,robot_def,[p[0],p[1],p[2]+0.15],R,vel_ctrl,distance_inst,robot_name,H_robot)
	#move down
	q=inv.inv(np.array([p[0],p[1],p[2]]),R)
	print(q)
	jog_joint(q)

	time.sleep(0.3)

	#grab it
	gripper.gripper(robot,True)
	# gripper.close()
	gripper_on=True
	print("get it")
	q=inv.inv(np.array([p[0],p[1],p[2]+0.15]),R)
	jog_joint(q)
	return

def place(obj,slot_name):
	global gripper_on

	obj_place_height=place_height+testbed_yaml[obj.name]+0.025

	#coordinate conversion
	print("placing at "+slot_name)
	wire_packet=detection_wire.TryGetInValue()
	slot=wire_packet[1][slot_name]
	capture_time=float(wire_packet[2].seconds)+float(wire_packet[2].nanoseconds*1e-9)

	p=conversion(slot.x,slot.y,obj_place_height)

	print(p)
	#get correct orientation

	R=R_ee.R_ee(angle_threshold(np.radians(slot.angle)-gripper_orientation))
	
	box_displacement=[[0],[0],[0]]

	plan.plan(robot,robot_def,[p[0],p[1],p[2]+0.15],R,vel_ctrl,distance_inst,robot_name,H_robot,obj_vel=obj_vel,capture_time=capture_time)


	box_displacement=obj_vel*(0.6+time.time()-capture_time)
	q=inv.inv(np.array([p[0]+box_displacement[0],p[1]+box_displacement[1],p[2]]),R)


	# jog_joint(q,(q-vel_ctrl.joint_position())/0.6)
	jog_joint(q,0.6)
	time.sleep(0.2)	#avoid inertia
	print("dropped")
	gripper.gripper(robot,False)
	# gripper.open()
	gripper_on=False
	
	q=inv.inv(np.array([p[0]+box_displacement[0],p[1]+box_displacement[1],p[2]+0.15]),R)
	jog_joint(q)
	return


def main():
	# gripper.initialize(robot)
	# time.sleep(2)

	#open gripper
	gripper.gripper(robot,False)

	#wait until wire value set
	while True:
		if detection_wire.TryGetInValue()[0]:
			break


	obj_grabbed=None
	action_performed=True
	while True:
		if action_performed:
			print('going home')
			single_move(home)
			action_performed=False


		if not gripper_on:

			for obj_name in obj_namelists:
				#check current robot free, and pick the object
				obj=detection_wire.InValue[obj_name]
				#pick obj&slot both detected object with priority
				if obj.detected and detection_wire.InValue[obj.name[0]+'_f'].detected:
					pick(obj)
					obj_grabbed=obj
					action_performed=True
					break
			#if no slots detected, pick up available object first
			if not gripper_on:
				for obj_name in obj_namelists:
					obj=detection_wire.InValue[obj_name]
					if obj.detected:
						pick(obj)
						obj_grabbed=obj
						action_performed=True
						break

		if gripper_on:
			slot=detection_wire.InValue[obj_grabbed.name[0]+'_f']
			#check slot is available and ready to drop
			if slot.detected:
				try:
					place(obj_grabbed,obj_grabbed.name[0]+'_f')
					action_performed=True
				#if out of workspace
				except ValueError:
					pass
				except UnboundLocalError:
					pass
				except:
					traceback.print_exc()



if __name__ == '__main__':
	main()