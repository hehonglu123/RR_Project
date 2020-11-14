#gripper helper function for UR
import time
from RobotRaconteur.Client import *

robot=RRN.ConnectService('rr+tcp://[fe80::76d6:e60f:27f6:1e3e]:58653/?nodeid=55ade648-a8c2-4775-a7ec-645acea83525&service=robot')

def close():	
	robot.setf_signal("DO5",1)
	robot.setf_signal("DO4",0)
	robot.setf_signal("DO3",1)
	while robot.getf_signal("AI0")<3.6:
		time.sleep(0.01)
	robot.setf_signal("DO5",0)
	robot.setf_signal("DO3",0)
def open():
	robot.setf_signal("DO5",1)
	robot.setf_signal("DO4",1)
	while robot.getf_signal("AI0")>3.1:
		time.sleep(0.01)
	robot.setf_signal("DO5",0)
	robot.setf_signal("DO4",0)	

def gripper(robot,on):
	
	if on:
		robot.setf_signal("DO5",1)
		robot.setf_signal("DO4",0)
		robot.setf_signal("DO3",1)
		while robot.getf_signal("AI0")<3.6:
			time.sleep(0.01)
		robot.setf_signal("DO5",0)
		robot.setf_signal("DO3",0)

	else:

		robot.setf_signal("DO5",1)
		robot.setf_signal("DO4",1)
		while robot.getf_signal("AI0")>2.8:
			time.sleep(0.01)
		robot.setf_signal("DO5",0)
		robot.setf_signal("DO4",0)	