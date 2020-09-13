import numpy as np
import yaml, math
from RobotRaconteur.Client import *
import sys, time
from scipy.optimize import leastsq
sys.path.append('../../../')
from vel_emulate import EmulatedVelocityControl


def my_func(x,obj,ref):

	R=np.array([[np.cos(x)[0],-np.sin(x)[0]],[np.sin(x)[0],np.cos(x)[0]]])

	result=np.dot(R,ref)-obj
	return result.flatten()


def calibrate(obj,ref):	
	

	return leastsq(func=my_func,x0=3.14,args=(np.transpose(np.array(obj)),np.transpose(np.array(ref))))

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
	

num_joints=6
#connect to wires
detection_wire=cognex_inst.detection_wire.Connect()
robot_state = robot.robot_state.Connect()
cmd_w = robot.position_command.Connect()

##############robot param setup
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
position_mode = robot_const["RobotCommandMode"]["position_command"]

robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = jog_mode
n= len(robot.robot_info.joint_info)

#######move to start point
robot.jog_joint(inv([0.35,-0.3,0.3]).reshape((n,1)), np.ones((n,)), False, True)
print("moving to start point")

#initialize coordinate list
robot_eef_coordinates=[[float(robot_state.InValue.kin_chain_tcp['position']['x'][0]),float(robot_state.InValue.kin_chain_tcp['position']['y'][0])]]
cam_coordinates=[[detection_wire.InValue[key].x,detection_wire.InValue[key].y]]


robot.command_mode = halt_mode
robot.command_mode = position_mode

vel_ctrl = EmulatedVelocityControl(robot,robot_state, cmd_w, 0.01)
vel_ctrl.enable_velocity_mode()
###
now=time.time()
timestamp=None

while time.time()-now<5:
	qdot=[0.2]+[0]*(num_joints-1)
	vel_ctrl.set_velocity_command(np.array(qdot))

	cognex_wire=detection_wire.TryGetInValue()
	if cognex_wire[0] and cognex_wire[2]!=timestamp:
		timestamp=cognex_wire[2]
		val1=float(robot_state.InValue.kin_chain_tcp['position']['x'][0])
		val2=float(robot_state.InValue.kin_chain_tcp['position']['y'][0])
		robot_eef_coordinates.append([val1,val2])
		cam_coordinates.append([detection_wire.InValue[key].x,detection_wire.InValue[key].y])
		

with open('camera.yaml','w') as file:
	yaml.dump({'cam_coordinates':cam_coordinates},file)
with open('robot.yaml','w') as file:
	yaml.dump({'robot_eef_coordinates':robot_eef_coordinates},file)


print(len(cam_coordinates))
result,res=calibrate(cam_coordinates, robot_eef_coordinates)
print(np.degrees(result))
R=np.array([[np.cos(result)[0],-np.sin(result)[0]],[np.sin(result)[0],np.cos(result)[0]]])
print(np.dot(R,np.array([[robot_state.InValue.kin_chain_tcp['position']['x'][0]],[robot_state.InValue.kin_chain_tcp['position']['y'][0]]]))-np.array([[detection_wire.InValue[key].x],[detection_wire.InValue[key].y]]))



vel_ctrl.set_velocity_command(np.zeros((num_joints,)))
vel_ctrl.disable_velocity_mode() 

