from RobotRaconteur.Client import *
import math
import numpy as np
import time
server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
w=server.get_worlds(str(server.world_names[0]))

#read sdf file
model_name="perfume"
f = open('../models/'+model_name+'/model.sdf','r')
sdff = f.read()
#generate RR pose
pose_dtype = RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose", server)
model_pose = np.zeros((1,), dtype = pose_dtype)
model_pose["orientation"]["w"] = 1.0
model_pose["position"]["x"] = np.random.uniform(-2,2,1),
model_pose["position"]["y"] = np.random.uniform(-2,2,1),
model_pose["position"]["z"] = np.random.uniform(0,3,1),
####insert a model
w.insert_model(sdff, "my_perfume", model_pose)
time.sleep(2)
m=w.get_models('my_perfume')
cur_pose=m.world_pose.PeekInValue()[0]
print(cur_pose['position'])


angle = 0
radius = 2
vel = 0.1	
now=time.time()
while True:
	pose=m.world_pose.PeekInValue()[0]
	pose_next=pose
	pose_next['position']['x']= radius * math.cos(angle * math.pi / 180)
	pose_next['position']['y']= radius * math.sin(angle * math.pi / 180)
	angle = angle + (vel / radius) * 180 / (math.pi * 30)
	if(angle == 360):
		angle = 0

	pose_next['position']['z'] = 0
	m.setf_world_pose(pose_next)
	if (time.time()-now)>5:
		break

# time.sleep(5)
w.remove_model("my_perfume")

