from RobotRaconteur.Client import *
import sys, time
#read in robot name and import proper libraries
if (sys.version_info > (3, 0)):
	robot_name=input('robot name: ')
else:
	robot=raw_input('robot name: ')

#auto discovery
time.sleep(2)
res=RRN.FindServiceByType("com.robotraconteur.robotics.robot.Robot",
["rr+local","rr+tcp","rrs+tcp"])
url=None
for serviceinfo2 in res:
	if robot_name in serviceinfo2.NodeName:
		url=serviceinfo2.ConnectionURL
		break
if url==None:
	print('service not found')
	sys.exit()

robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)

state_w = robot_sub.SubscribeWire("robot_state")

while True:
	if state_w.TryGetInValue()[0]:
		print(state_w.InValue.joint_position_command)
	time.sleep(0.1)