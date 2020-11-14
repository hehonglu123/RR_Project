#!/usr/bin/env python3
import numpy as np
from RobotRaconteur.Client import *
import sys, time, yaml, argparse
from scipy.optimize import leastsq
from importlib import import_module
sys.path.append('../')
from vel_emulate_sub import EmulatedVelocityControl
from jog_joint import jog_joint

sys.path.append('../toolbox')
from general_robotics_toolbox import Robot, fwdkin
from autodiscovery import autodiscover


def my_func(x,obj,ref):

    R=np.array([[np.cos(x[0]),-np.sin(x[0])],[np.sin(x[0]),np.cos(x[0])]])
    result=np.dot(R,ref)-obj+np.array([[x[1]],[x[2]]])
    return result.flatten()

def calibrate(obj,ref): 
    result,r = leastsq(func=my_func,x0=[0,0,0],args=(np.transpose(np.array(obj)),np.transpose(np.array(ref))))
    H=np.zeros((4,4))
    H[0][0]=np.cos(result[0])
    H[0][1]=-np.sin(result[0])
    H[1][0]=np.sin(result[0])
    H[1][1]=np.cos(result[0])
    H[2][2]=1
    H[0][-1]=result[1]
    H[1][-1]=result[2]
    H[-1][-1]=1
    return H



key="eef"
# R=[[1,0,0],[0,1,0],[0,0,1]]
#input robot name
#connection failed callback
def connect_failed(s, client_id, url, err):
    print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))

#Accept the names of the webcams and the nodename from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--robot-name",type=str,help="List of camera names separated with commas")
args, _ = parser.parse_known_args()

robot_name=args.robot_name


sys.path.append('../toolbox')
inv = import_module(robot_name+'_ik')
R_ee = import_module('R_'+robot_name)

#########read in yaml file for robot client
with open(r'../client_yaml/client_'+robot_name+'.yaml') as file:
    robot_yaml = yaml.load(file, Loader=yaml.FullLoader)


####################Start Service and robot setup
###########Connect to corresponding services, subscription mode
####subscription
# time.sleep(2)
# url=autodiscover("edu.rpi.robotics.cognex.cognex","cognex")
# if url==None:
#     print("service not found")
#     sys.exit(1)
url='rr+tcp://[fe80::922f:c9e6:5fe5:51d1]:52222/?nodeid=87518815-d3a3-4e33-a1be-13325da2461f&service=cognex'
cognex_sub=RRN.SubscribeService(url)

robot_sub=RRN.SubscribeService(robot_yaml['url'])
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

##########Initialize robot parameters   #need modify
num_joints=len(robot.robot_info.joint_info)
P=np.array(robot.robot_info.chains[0].P.tolist())
length=np.linalg.norm(P[1])+np.linalg.norm(P[2])+np.linalg.norm(P[3])
H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))

P[-1]+=robot_yaml['tag_position']

robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))


#######move to start point
print("moving to start point")
R=np.array(robot_yaml['calibration_R']).reshape((3,3))
print(R)
start_joints=inv.inv(robot_yaml['calibration_start'],R)
# robot.command_mode = jog_mode 
# robot.jog_joint(start_joints,np.ones((num_joints,)), False, True)
# time.sleep(5)

robot.command_mode = halt_mode 
robot.command_mode = position_mode 
vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w, 0.01)
jog_joint(robot,vel_ctrl,start_joints,5)


#enable velocity mode
vel_ctrl.enable_velocity_mode()

#initialize coordinate list
joint_angles=[state_w.InValue.joint_position]
cam_coordinates=[[detection_wire.InValue[key].x,detection_wire.InValue[key].y]]

print("calibrating")
timestamp=None
now=time.time()
while time.time()-now<60:
    qdot=[robot_yaml['calibration_speed']]+[0]*(num_joints-1)
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
    if robot_name=='ur':
        joint_angles[i][0]+=np.pi
    transform=fwdkin(robot_def,joint_angles[i])
    p=transform.p
    eef.append(p.tolist()[:2])
H=calibrate(cam_coordinates, eef)
H[2][-1]=robot_yaml['height']
print(H)
dict_file={'H':H.tolist()}
# directory='/home/rpi/RR_Project/calibration'
# os.chdir(directory)
with open('/home/rpi/RR_Project/calibration/'+robot_name+'.yaml', 'w') as file:
    yaml.dump(dict_file, file)

