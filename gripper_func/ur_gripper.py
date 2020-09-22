#gripper helper function for UR
import time
def gripper(robot,on):
	if on:
		robot.setf_signal("D5",1)
		robot.setf_signal("D3",1)
		time.sleep(0.3)
		robot.setf_signal("D5",0)
		robot.setf_signal("D3",0)

	else:
		robot.setf_signal("D5",1)
		robot.setf_signal("D4",1)
		time.sleep(0.3)
		robot.setf_signal("D5",0)
		robot.setf_signal("D4",0)
	