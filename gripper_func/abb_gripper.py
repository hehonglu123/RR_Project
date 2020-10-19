#gripper helper function for ABB
from RobotRaconteur.Client import *
def gripper(robot,on):
	robot=RRN.ConnectService('rr+tcp://[fe80::76d6:e60f:27f6:1e3e]:58653/?nodeid=55ade648-a8c2-4775-a7ec-645acea83525&service=robot')
	robot.setf_signal("DO7",on)
