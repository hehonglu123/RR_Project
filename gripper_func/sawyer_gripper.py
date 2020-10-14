#gripper helper function for sawyer
def initialize(robot):
	robot.setf_signal("right_valve_1a",0)
	robot.setf_signal("right_valve_1b",1)

def gripper(robot,on):

	robot.setf_signal("right_valve_1a",on)
	robot.setf_signal("right_valve_1b",not on)
