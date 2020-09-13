from scipy.optimize import leastsq
import numpy as np
import yaml



def calc_error(robot,cam,theta):
	R=np.array([[np.cos(theta)[0],-np.sin(theta)[0]],[np.sin(theta)[0],np.cos(theta)[0]]])
	error=np.dot(R,np.transpose(np.array(cam)))-np.transpose(np.array(robot))
	return np.average(error,axis=0)

with open('robot.yaml') as file1:
	robot = yaml.load(file1)['robot_eef_coordinates']
with open('camera.yaml') as file2:
	cam=yaml.load(file2)['cam_coordinates']

# print(calc_error(robot,cam,3.14))