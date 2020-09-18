#gripper helper function for UR

def gripper(robot,on):
	if on:
		robot.setf_signal("D4",not on)
		robot.setf_signal("D5",on)
		robot.setf_signal("D3",on)
	else:
		robot.setf_signal("D5",on)
		robot.setf_signal("D3",on)
		robot.setf_signal("D4",not on)
	