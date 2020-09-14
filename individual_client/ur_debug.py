from RobotRaconteur.Client import *
import sys, time, yaml, traceback
import numpy as np
sys.path.append('../')
from vel_emulate_sub import EmulatedVelocityControl
sys.path.append('../toolbox')
from general_robotics_toolbox import *      

robot_name='ur'
url='rr+tcp://bbb2.local:58652?service=ur_robot'
robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
robot.command_mode = halt_mode

state_w = robot_sub.SubscribeWire("robot_state")


##########Initialize robot parameters	#need modify
num_joints=len(robot.robot_info.joint_info)
P=np.array(robot.robot_info.chains[0].P.tolist())
length=np.linalg.norm(P[1])+np.linalg.norm(P[2])+np.linalg.norm(P[3])
H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
# joint_type = robot.robot_info.joint_info.joint_type.tolist()
robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))

robot_joints=state_w.InValue.joint_position
robot_joints[0]+=np.pi
print(fwdkin(robot_def,robot_joints))

