#gripper helper function for UR

def gripper(robot,on):
	robot.setf_signal("D07",on)