
from RobotRaconteur.Client import *
import numpy as np
import time
import traceback


testbed_inst=RRN.ConnectService('rr+tcp://localhost:6666?service=testbed')   #testbed service

try:
	print(testbed_inst.boxes[-1].name)
	print(testbed_inst.boxes[-1].filled)
	testbed_inst.modify_box(-1,2,"bottle")

	print(testbed_inst.boxes[-1].onboard)
	print(testbed_inst.boxes[-1].filled)
	print(testbed_inst.boxes[-1].name)
except:
	traceback.print_exc()