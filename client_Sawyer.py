#Simple example Robot Raconteur Sawyer Robot and Cognex client
#pick up and drop detected objects
from RobotRaconteur.Client import *
import numpy as np
import time
import traceback
import copy
import sys
from vel_emulate import EmulatedVelocityControl
sys.path.insert(1,"toolbox")
from sawyer_ik import fwd, ik
sys.path.insert(1,"QP_planner")
from plan_Sawyer import plan



####################Start Service and robot setup
inst=RRN.ConnectService('rr+tcp://128.113.224.144:52222/?service=SmartCam',"cats",{"password":RR.RobotRaconteurVarValue("cats111!","string")},None,None)
robot = RRN.ConnectService('rr+tcp://127.0.0.1:58653?service=sawyer')
robot2 = RRN.ConnectService('rr+tcp://localhost:58652?service=ur_robot')
distance_inst=RRN.ConnectService('rr+tcp://127.0.0.1:25522?service=Environment')


robot.enable()
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
robot.command_mode = halt_mode
time.sleep(0.1)

###velocity control parameters
cmd_w = robot.position_command.Connect()
state_w = robot.robot_state.Connect()
RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",robot)
vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w, 0.01)


####load parameters
obj_namelists=['sp','bt']
transformations=distance_inst.transformations
H=transformations[1].H.reshape((transformations[1].row,transformations[1].column))


Rd=np.array([[ 0., 0., -1. ],
 [ 0., -1.,  0.],
 [-1.,  0., 0.]])

def joint_threshold(q):
	if q[-1][0]>np.pi:
		q[-1][0]-=2*np.pi
	elif q[-1][0]<-np.pi:
		q[-1][0]+=2*np.pi
	return q

def vacuum(robot,on):
	if on:
		robot.setf_signal("right_valve_1a",0)
		robot.setf_signal("right_valve_1b",1)
	else:
		robot.setf_signal("right_valve_1a",1)
		robot.setf_signal("right_valve_1b",0)

def moveup(robot):
	robot_state = robot.robot_state.PeekInValue()
	position=robot_state[0].kin_chain_tcp[0]['position'] 
	robot_joints=robot_state[0].joint_position
	robot.command_mode = halt_mode
	time.sleep(0.1)
	robot.command_mode=jog_mode
	p=ik(Rd,[position[0],position[1],position[2]+0.05])
	p[-1]=robot_joints[-1]
	robot.jog_joint(p, np.ones((7,)), False, True)

def single_move(p,robot):
	global vel_ctrl, state_w, vel_ctrl
	robot.command_mode = halt_mode
	time.sleep(0.1)
	robot.command_mode = position_mode 
	robot_state = robot.robot_state.PeekInValue()
	robot_joints=robot_state[0].joint_position.reshape((7,1)) 
	p1=ik(Rd,p).reshape((7,1))

	Q_all = np.array([robot_joints,p1])
	plan(robot,robot2,Q_all, vel_ctrl,1)
	return
def move(p,robot,action,angle=0):				#go up, go destination, go down, action: 1 grab, 0 release
	global Rd, state_w, vel_ctrl
	robot.command_mode = halt_mode
	time.sleep(0.1)
	robot.command_mode = position_mode 

	robot_state = robot.robot_state.PeekInValue()
	position=robot_state[0].kin_chain_tcp[0]['position'] 
	robot_joints=robot_state[0].joint_position.reshape((7,1)) 

	angle=np.radians(angle)-np.pi/2
	if (angle<-np.pi):
		angle+=2*np.pi


	R=np.array([[0, -np.sin(angle),-np.cos(angle)],
				[0, -np.cos(angle), np.sin(angle)],
				[-1,0,0]])

	R_cur=fwd(robot_joints).R
	
	p1=ik(R_cur,np.array([position[0],position[1],position[2]+0.1])).reshape((7,1))
	p2=ik(R,np.array([p[0],p[1],p[2]+0.1])).reshape((7,1))
	p3=ik(R,np.array([p[0],p[1],p[2]])).reshape((7,1))

	p2[-1][0]=p2[0][0]-angle
	p2=joint_threshold(p2)
	p3[-1][0]=p3[0][0]-angle
	p3=joint_threshold(p3)


	if(position[1]>0.4):
		p_temp=ik(R,np.array([position[0],0.25,p[2]+0.1])).reshape((7,1))
	else:
		p_temp=p2
	if action==1:
		p1=ik(R_cur,np.array([position[0]-0.05,position[1],position[2]+0.1])).reshape((7,1))
		p_temp=p_temp=ik(R,np.array([0.15,0.45,0.2])).reshape((7,1))
	try:
		Q_all = np.array([robot_joints,p1,p_temp,p2])
		print("velocity move")
		plan(robot,robot2,Q_all,vel_ctrl,action)
	except:
		traceback.print_exc()
	#move down through jog joint
	print("moving down")
	try:
		robot.command_mode = halt_mode
		time.sleep(0.1)
		robot.command_mode=jog_mode

		robot.jog_joint(p3, np.ones((7,)), False, True)
		# time.sleep(1)

	except:
		traceback.print_exc()
	print("get it")
	vacuum(robot,action)
	return

def move_obj(obj,destination,H,robot):
	try:
		if obj.name=="sp" and destination[0][0]!=0 and destination[0][0]!=1:
			print("moving soap bar")
			p=np.dot(H,np.array([[obj.x/1000.],[obj.y/1000.],[1]]))
			p[1][0]+=0.01							#distortion offset
			p[0][0]+=0.008							#distortion offset
			p[2][0]=0.107							#soap bar height
			print("obj at: ",p)
			move(p.reshape(3),robot,1,obj.angle)
			print("moving to destination")
			p=np.dot(H,np.array([[destination[0][0]],[destination[0][1]],[1]]))
			p[2][0]=0.05
			p[0][0]+=0.005							#distortion offset
			p[1][0]-=0.008							#distortion offset
			move(p.reshape(3),robot,0,destination[0][2])
			destination[0]=np.ones(1)
			return destination

		if obj.name=="bt" and destination[1][0]!=0 and destination[1][0]!=1:
			print("moving bottle")
			p=np.dot(H,np.array([[obj.x/1000.],[obj.y/1000.],[1]]))
			p[0][0]+=0.01							#distortion offset
			p[2][0]=0.148#+0.05						#bottle height
			print("obj at: ",p)
			move(p.reshape(3),robot,1)
			print("moving to destination")
			p=np.dot(H,np.array([[destination[1][0]],[destination[1][1]],[1]]))
			p[2][0]=0.09
			p[0][0]+=0.005							#distortion offset
			p[1][0]-=0.008							#distortion offset
			move(p.reshape(3),robot,0)
			destination[1]=np.ones(1)
			return destination
	except:
		return 

vacuum(robot,0)
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode=jog_mode
robot.jog_joint(ik(Rd,[0.35,-0.3,0.25]).reshape((7,1)), np.ones((7,)), False, True)

new_box=1
destination=np.zeros((2,3))
while True:
	new_box_count=0
	for obj in inst.objects:
		if "s_f" == obj.name:
			new_box_count+=1
		elif "b_f" == obj.name:
			new_box_count+=1
		elif "p_f" == obj.name:
			new_box_count+=1
		elif "t_f" == obj.name:
			new_box_count+=1
	if new_box_count==4:
		new_box=1
	else:
		new_box=0

	if new_box:
		destination=np.zeros((2,3))	#0 soap, 1 bottle
		for obj in inst.objects:
			try:
				if obj.name=="s_f" and obj.x>0:
					destination[0][0]=obj.x/1000.
					destination[0][1]=obj.y/1000.
					destination[0][2]=obj.angle

				if obj.name=="b_f":
					destination[1][0]=obj.x/1000.
					destination[1][1]=obj.y/1000.
					destination[1][2]=obj.angle

			except:
				pass

	for obj in inst.objects:
		if obj.name in obj_namelists:
			print(obj.name+" detected")
			print("destination: ",destination)
			destination=move_obj(obj,destination,H,robot)

	robot_state = robot.robot_state.PeekInValue()
	position=robot_state[0].kin_chain_tcp[0]['position'] 

	if (position[2]<0.1):
		moveup(robot)

	robot.command_mode = halt_mode
	time.sleep(0.1)
	robot.command_mode = jog_mode
	robot.jog_joint(ik(Rd,[0.3,-0.3,0.2]).reshape((7,1)), np.ones((7,)), False, True)
