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
from inv_kin import lab_invk
sys.path.insert(1,"QP_planner")
from plan_UR import *

####################Start Service and robot setup
inst=RRN.ConnectService('rr+tcp://128.113.224.144:52222/?service=SmartCam',"cats",{"password":RR.RobotRaconteurVarValue("cats111!","string")},None,None)
robot = RRN.ConnectService('rr+tcp://localhost:58652?service=ur_robot')
robot2= RRN.ConnectService('rr+tcp://127.0.0.1:58653?service=sawyer')
distance_inst=RRN.ConnectService('rr+tcp://127.0.0.1:25522?service=Environment')
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


####load homogeneous transformation parameters Cognex->robot
obj_namelists=['pf','tp']
transformations=distance_inst.transformations
H=transformations[0].H.reshape((transformations[0].row,transformations[0].column))


def vacuum(robot,on):
	robot.setf_signal("D07",on)
def moveup(robot):
	global state_w
	robot_state = state_w.InValue
	position=robot_state.kin_chain_tcp[0]['position'] 
	robot_joints=robot_state.joint_position
	robot.command_mode = halt_mode
	time.sleep(0.05)
	robot.command_mode=jog_mode
	p=lab_invk([position[0],position[1],position[2]+0.05]).reshape((6,1))
	p[-1]=robot_joints[-1]
	robot.jog_joint(p, np.ones((6,)), False, True)

def single_move(p,robot):
	global state_w, vel_ctrl
	robot.command_mode = halt_mode
	time.sleep(0.1)
	robot.command_mode = position_mode 
	robot_state = state_w.InValue
	robot_joints=robot_state.joint_position.reshape((6,1)) 
	p1=lab_invk(p).reshape((6,1))
	Q_all = np.array([robot_joints,p1])
	plan(robot,robot2,Q_all, vel_ctrl, 1)
	return
def move(p,robot,action,angle=0):				#go up, go destination, go down, action: 1 grab, 0 release
	global Rd, state_w, vel_ctrl
	time.sleep(1)
	print("switching halt mode")
	robot.command_mode = halt_mode
	time.sleep(0.1)
	print("switching position mode")
	robot.command_mode = position_mode 

	robot_state = state_w.InValue
	position=robot_state.kin_chain_tcp[0]['position'] 
	robot_joints=robot_state.joint_position.reshape((6,1)) 

	
	p1=lab_invk(np.array([position[0],position[1],position[2]+0.1])).reshape((6,1))
	p2=lab_invk(np.array([p[0],p[1],p[2]+0.1]),orientation=angle).reshape((6,1))
	p3=lab_invk(np.array([p[0],p[1],p[2]]),orientation=angle).reshape((6,1))


	Q_all = np.array([robot_joints,p1,p2])
	print(p1)
	print(p2)
	print(p3)
	try:
		plan(robot,robot2,Q_all, vel_ctrl, action)
	except:
		traceback.print_exc()
	#move down through jog joint
	try:
		
		time.sleep(1)
		print("switching halt mode")
		robot.command_mode = halt_mode
		time.sleep(0.1)
		print("switching jog mode")
		robot.command_mode=jog_mode
		robot.jog_joint(p3, np.ones((6,)), False, True)

	except:
		traceback.print_exc()
	print("get it")
	vacuum(robot,action)
	return

def move_obj(obj,destination,H,robot):
	try:
		if obj.name=="tp" and destination[0][0]!=0 and destination[0][0]!=1:
			print("moving toothpaste")
			p=np.dot(H,np.array([[obj.x/1000.],[obj.y/1000.],[1]]))
			p[2][0]=0.04						#toothpaste height
			print("obj at: ",p)
			move(p.reshape(3),robot,1,obj.angle)
			print("moving to destination")
			p=np.dot(H,np.array([[destination[0][0]],[destination[0][1]],[1]]))
			p[2][0]=0.
			p[0][0]+=0.005						#distortion

			move(p.reshape(3),robot,0,destination[0][2])
			destination[0]=np.ones(1)
			return destination

		if obj.name=="pf" and destination[1][0]!=0 and destination[1][0]!=1:
			print("moving perfume")
			p=np.dot(H,np.array([[obj.x/1000.],[obj.y/1000.],[1]]))
			p[2][0]=0.04						#perfume bottle height
			print("obj at: ",p)
			
			move(p.reshape(3),robot,1,obj.angle)
			print("moving to destination")
			p=np.dot(H,np.array([[destination[1][0]],[destination[1][1]],[1]]))
			p[2][0]=0.

			move(p.reshape(3),robot,0,destination[1][2])
			destination[1]=np.ones(1)
			return destination
	except:
		return 

vacuum(robot,0)
single_move([-0.4,0.1,0.2],robot)
# time.sleep(15)
destination=np.zeros((2,3))
new_box=1

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
		print("new_box")
		new_box=1
	else:
		new_box=0
	if new_box:
		destination=np.zeros((2,3))	#0 toothpaste, 1 perfume
		for obj in inst.objects:
			try:
				if obj.name=="t_f" and obj.x>0:
					destination[0][0]=obj.x/1000.
					destination[0][1]=obj.y/1000.
					destination[0][2]=obj.angle
					if np.linalg.norm(destination[0][:-1]-destination_prev[0][:-1])<1:
						destination[0]=np.zeros(3)

				if obj.name=="p_f":
					destination[1][0]=obj.x/1000.
					destination[1][1]=obj.y/1000.
					destination[1][2]=obj.angle

					if np.linalg.norm(destination[1][:-1]-destination_prev[1][:-1])<1:
						destination[1]=np.zeros(3)
			except:
				pass

	for obj in inst.objects:
		if obj.name in obj_namelists:
			print(obj.name+" detected")
			print(destination)
			print(obj.x)
			print(obj.y)
			print(obj.angle)
			destination=move_obj(obj,destination,H,robot)

	robot_state = state_w.InValue
	position=robot_state.kin_chain_tcp[0]['position'] 
	if (position[2]<0.1):
		moveup(robot)

	single_move([-0.4,0.12,0.3],robot)



