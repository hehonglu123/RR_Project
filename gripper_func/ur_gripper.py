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
		time.sleep(0.38)
		robot.setf_signal("DO5",0)
		robot.setf_signal("DO3",0)

	else:
		#neutral point release
		robot.setf_signal("DO5",0)
		robot.setf_signal("DO4",1)
		robot.setf_signal("DO3",0)
		
		# # timed release
		# robot.setf_signal("DO5",1)
		# robot.setf_signal("DO4",1)
		# time.sleep(0.1)
		# robot.setf_signal("DO5",0)
		# robot.setf_signal("DO4",0)
	