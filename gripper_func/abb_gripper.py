#gripper helper function for ABB
from RobotRaconteur.Client import *
def gripper(robot,on):
	robot=RRN.ConnectService('rr+tcp://[fe80::76d6:e60f:27f6:1e3e]:58652?service=ur_robot')
	robot.setf_signal("D7",on)
