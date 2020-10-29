#gripper helper function for ABB
from RobotRaconteur.Client import *
def gripper(robot,on):
	abb_gripper=RRN.ConnectService('rr+tcp://localhost:11222?service=abb_gripper')
	if on:
		abb_gripper.close()
	else:
		abb_gripper.open()

