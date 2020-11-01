#gripper helper function for UR
import time
def initialize(robot):
	#neutral point release
	robot.setf_signal("DO5",0)
	robot.setf_signal("DO4",1)
	robot.setf_signal("DO3",0)

def gripper(robot,on):
	
	if on:
		robot.setf_signal("DO5",1)
		robot.setf_signal("DO4",0)
		robot.setf_signal("DO3",1)
		while robot.getf_signal("AI0")<3.6:
			time.sleep(0.01)
		robot.setf_signal("DO5",0)
		robot.setf_signal("DO3",0)

	else:

		robot.setf_signal("DO5",1)
		robot.setf_signal("DO4",1)
		while robot.getf_signal("AI0")>2.8:
			time.sleep(0.01)
		robot.setf_signal("DO5",0)
		robot.setf_signal("DO4",0)	