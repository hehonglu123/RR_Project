#gripper helper function for ABB
from RobotRaconteur.Client import *
def gripper(robot,on):
	robot=RRN.ConnectService('rr+tcp://[fe80::a2c:1efa:1c07:f043]:58654/?nodeid=8edf99b5-96b5-4b84-9acf-952af15f0918&service=robot')
	robot.setf_signal("DO7",on)
