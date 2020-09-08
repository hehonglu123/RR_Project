#gripper helper function for sawyer
def gripper(robot,on):
	if on:
		robot.setf_signal("right_valve_1a",0)
		robot.setf_signal("right_valve_1b",1)
	else:
		robot.setf_signal("right_valve_1a",1)
		robot.setf_signal("right_valve_1b",0)