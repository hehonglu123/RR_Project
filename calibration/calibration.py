import numpy as np
import yaml, sys
from RobotRaconteur.Client import *
def calibrate(obj,ref):	#list of [x_c,y_c] and [x_r,y_r]
	obj=np.hstack((obj,np.ones((len(obj),1))))
	H,residuals,rank,s=np.linalg.lstsq(obj,ref,rcond=-1)
	return np.vstack((np.transpose(H),[0,0,1]))
def calibrate_plug(obj,ref,cam_origin):
	A=np.array([[obj[0],-obj[1]],[obj[1],obj[0]]])
	b=np.transpose(ref-cam_origin)
	print(A)
	print(b)
	temp=np.linalg.solve(A,b)
	a=temp[0]
	b=temp[1]
	H=np.array([[a,b,cam_origin[0]],[-b,a,cam_origin[1]],[0,0,1],])
	return H
#convert 3x3 H matrix to 4x4 H matrix 
def H32H4(H, height):
	H4=H[:2,:2]
	T=np.dot(np.linalg.inv(H4),np.array([[H[0][-1]],[H[1][-1]]]))
	H4=np.hstack((H4,[[0],[0]]))
	H4=np.vstack((H4,[[0,0,1]]))
	H4=np.hstack((H4,[[T[0][0]],[T[1][0]],[height]]))
	H4=np.vstack((H4,[0,0,0,1]))
	return H4


#read in robot name and import proper libraries
if (sys.version_info > (3, 0)):
	robot_name=input('robot name: ')
else:
	robot=raw_input('robot name: ')

with open(r'../client_yaml/client_'+robot_name+'.yaml') as file:
    robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
url=robot_yaml['url']
home=robot_yaml['home']
calibration_start=robot_yaml['calibration_start']
calibration_speed=robot_yaml['calibration_speed']
robot_height=robot_yaml['height']
filename=robot_name+'.yaml'

####subscription
cognex_sub=RRN.SubscribeService('rr+tcp://localhost:52222/?service=cognex')
robot_sub=RRN.SubscribeService(url)
####get client object
cognex_inst=cognex_sub.GetDefaultClientWait(1)
robot=robot_sub.GetDefaultClientWait(1)
####get subscription wire
#cognex detection wire
detection_wire=cognex_sub.SubscribeWire("detection_wire")



pose = robot.robot_state.PeekInValue()[0].kin_chain_tcp['position']

obj=[]
ref=[]
##########accurate version
# raw_input("please remove robot and place 3 objects")
# while (len(inst.objects)<3):
# 	print("number of objects detected: ",len(inst.objects))
# 	raw_input("please move robot away and place 3 objects")
# detected_obj=inst.objects
# for i in range(3):
# 	obj.append([detected_obj[i].x/1000.,detected_obj[i].y/1000.])
# 	raw_input("please put robot endeffector on top of object: "+detected_obj[i].name)
# 	pose = robot.robot_state.PeekInValue()[0].kin_chain_tcp['position']
# 	ref.append([pose['x'][0],pose['y'][0]])
# H=calibrate(obj,ref)

###############plug and play version
input("please put robot endeffector on top of camera origin: ")
pose = robot.robot_state.PeekInValue()[0].kin_chain_tcp['position']
cam_origin=[pose['x'][0],pose['y'][0]]
bottle=cognex_inst.detection_wire.PeekInValue()[0]['tp']
input("please put robot endeffector on top of object: tp")
pose = robot.robot_state.PeekInValue()[0].kin_chain_tcp['position']
obj=np.array([bottle.x, bottle.y])
ref=np.array([pose['x'][0],pose['y'][0]])
H=calibrate_plug(obj,ref,cam_origin)

print(H)

H=H32H4(H,robot_height)
print(H)
dict_file={'H':H.tolist()}
with open(filename,'w') as file:
	yaml.dump(dict_file,file)

