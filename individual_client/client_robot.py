#Simple example Robot Raconteur Robot Print joint client
from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, argparse
sys.path.append('../toolbox')
from general_robotics_toolbox import q2R  

#connection failed callback
def connect_failed(s, client_id, url, err):
	print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))


#Accept the names of the webcams and the nodename from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--robot-name",type=str,help="List of camera names separated with commas")
args, _ = parser.parse_known_args()

robot_name=args.robot_name

time.sleep(2)
res=RRN.FindServiceByType("com.robotraconteur.robotics.robot.Robot",
["rr+local","rr+tcp","rrs+tcp"])

url=None
for serviceinfo2 in res:
	if robot_name in serviceinfo2.Name:
		url=serviceinfo2.ConnectionURL
		break
if url==None:
	print('service not found')
	sys.exit()

####################Start Service and robot setup

robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)

state_w = robot_sub.SubscribeWire("robot_state")

print('device name: ',robot.robot_info.device_info.device.name)

time.sleep(0.5)
robot_state_wire=state_w.TryGetInValue()
if robot_state_wire[0]:
	robot_state = robot_state_wire[1]
	print("robot_joints: ", robot_state.joint_position)
	
	position=robot_state.kin_chain_tcp[0]['position'] 
	orientation=robot_state.kin_chain_tcp[0]['orientation'] 
	print('eef position: ',position)
	print('eef orientation: ',q2R(list(orientation)))
else:
	print("wire value not set")
