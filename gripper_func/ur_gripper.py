#gripper helper function for UR

def gripper(robot,on):
	if on:
		robot.setf_signal("D04",not on)
		robot.setf_signal("D05",on)
		robot.setf_signal("D03",on)
	else:
		robot.setf_signal("D05",on)
		robot.setf_signal("D03",on)
		robot.setf_signal("D04",not on)
	