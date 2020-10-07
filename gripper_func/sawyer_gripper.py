#gripper helper function for sawyer
def gripper(robot,on):

	robot.setf_signal("right_valve_1a",on)
	robot.setf_signal("right_valve_1b",not on)
