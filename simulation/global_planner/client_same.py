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
cognex_sub=RRN.SubscribeService('rr+tcp://localhost:52222/?service=cognex')
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
##distance report wire
distance_report_wire=distance_sub.SubscribeWire("distance_report_wire")

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
jog_mode = robot_const["RobotCommandMode"]["jog"]

position_mode = robot_const["RobotCommandMode"]["position_command"]
trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]
robot.command_mode = halt_mode

##########Connect to Cognex wire
# ##########Initialize velocity control parameters
RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",robot)
vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w, 0.01)
robot.command_mode = trajectory_mode 

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

def exe_traj(traj):

	if len(traj.waypoints)<=4:
		return
	traj_gen = robot.execute_trajectory(traj)
	while (True):
		try:

			res = traj_gen.Next()

		except RR.StopIterationException:
			# jog_joint(traj.waypoints[-1].joint_position)
			distance_inst.clear_traj(robot_name)
			# print(np.linalg.norm(traj.waypoints[-1].joint_position-state_w.InValue.joint_position))
			
			return
def jog_joint_tracking(q):
	robot.command_mode = halt_mode
	time.sleep(0.02)
	robot.command_mode = position_mode
	#enable velocity mode
	vel_ctrl.enable_velocity_mode()
	# qdot=np.zeros(num_joints)
	
	while np.linalg.norm(q-vel_ctrl.joint_position())>0.1:
		qdot=2*(q-vel_ctrl.joint_position())
		# print(qdot)
		qdot[:-2]=np.array([x if np.abs(x)>0.1 else 0.1*np.sign(x) for x in qdot])[:-2]
		vel_ctrl.set_velocity_command(qdot)

	vel_ctrl.set_velocity_command(np.zeros((num_joints,)))
	vel_ctrl.disable_velocity_mode() 
	robot.command_mode = halt_mode
	time.sleep(0.01)
	robot.command_mode = trajectory_mode
	return

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
def single_move(p):
	traj=None
	while traj is None:
		try:
			traj=distance_inst.plan(robot_name,3.,p,list(R_ee.R_ee(0).flatten()),joint_threshold,[0.,0.,0.],0)

		except:
			print("replanning")
			time.sleep(0.2)
			traceback.print_exc()
			pass
	
	exe_traj(traj)
	
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
	print("moving "+obj.name)
	p=conversion(obj.x,obj.y,pick_height)							
	
	#move to object above
	traj=None
	while traj is None:
		try:
			traj=distance_inst.plan(robot_name,3.,[p[0],p[1],p[2]+0.1],list(R_ee.R_ee(0).flatten()),joint_threshold,[0.,0.,0.],0)
		except:
			print("replanning")
			time.sleep(0.2)
			pass

	exe_traj(traj)
	#move down
	q=inv.inv(np.array([p[0],p[1],p[2]]))
	jog_joint_tracking(q)

	#grab it
	print("get it")
	vacuum_inst.vacuum(robot_name,obj.name,1)
	q=inv.inv(np.array([p[0],p[1],p[2]+0.1]))
	jog_joint(q)
	return
def place(obj,slot_name):

	#coordinate conversion
	slot=detection_wire.InValue[slot_name]
	capture_time=time.time()
	p=conversion(slot.x,slot.y,place_height)
	#get correct orientation
	angle=(slot.angle-obj.angle)

	R=R_ee.R_ee(angle_threshold(np.radians(angle)))
	
	box_displacement=[[0],[0],[0]]
	jog_joint_time=1.
	traj=None
	while traj is None:
		try:
			traj=distance_inst.plan(robot_name,3.,[p[0],p[1],p[2]+0.15],list(R.flatten()),joint_threshold,list(obj_vel.flatten()),capture_time-jog_joint_time)
		except UnboundLocalError:
			raise UnboundLocalError
			return
		except:
			print("replanning")
			time.sleep(0.2)
			pass
	exe_traj(traj)


	box_displacement=obj_vel*(jog_joint_time+time.time()-capture_time)
	q=inv.inv(np.array([p[0]+box_displacement[0],p[1]+box_displacement[1],p[2]]),R)


	jog_joint_tracking(q)
	time.sleep(0.02)
	print("dropped")
	vacuum_inst.vacuum(robot_name,obj.name,0)

	testbed_inst.onboard=testbed_inst.onboard+[obj.name]
	temp=testbed_inst.filled
	temp[int(slot.box_idx*testbed_inst.num_slot+slot_dict[slot_name[-3:]])]=1
	testbed_inst.filled=temp
	
	q=inv.inv(np.array([p[0]+box_displacement[0],p[1]+box_displacement[1],p[2]+0.1]),R)
	jog_joint(q)
	return


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

	if vacuum_inst.actions[robot_name]!=1:
		for i in range(len(obj_namelists)):
			#check current robot free, and pick the object
			obj=detection_wire.InValue[obj_namelists[i]]
			if obj.detected:
				pick(obj)
				action_performed=True
				obj_grabbed=obj
				break
	for j in range(testbed_inst.num_box):
		slot=detection_wire.InValue['box'+str(j)+obj_grabbed.name[0]+'_f']
		#check slot is available and ready to drop
		if slot.detected and vacuum_inst.actions[robot_name]==1:
			try:
				place(obj_grabbed,'box'+str(j)+obj_grabbed.name[0]+'_f')
				action_performed=True
			except ValueError:
				pass
			except UnboundLocalError:
				pass
			except:
				traceback.print_exc()