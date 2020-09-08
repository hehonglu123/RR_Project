#Simple example Robot Raconteur Sawyer Robot and Cognex client
#pick up and drop detected objects
from RobotRaconteur.Client import *

import numpy as np
import time
import traceback
import sys



####################Start Service and robot setup
url='rr+tcp://localhost:52222/?service=cognexsim'

sub=RRN.SubscribeService(url)
while True:
	try:
		obj = sub.GetDefaultClient()
		detection_wire=sub.SubscribeWire("detection_wire")
		break
	except RR.ConnectionException:
		time.sleep(0.1)
while True:
	if detection_wire.TryGetInValue()[0]:
		print(detection_wire.InValue['box0b_f'].angle)
		print(detection_wire.InValue['box0t_f'].angle)
		# print("ur_eef: ",detection_wire.InValue['ur_eef'].x,detection_wire.InValue['ur_eef'].y,detection_wire.InValue['ur_eef'].angle)
		# print("sawyer_eef: ",detection_wire.InValue['sawyer_eef'].x,detection_wire.InValue['sawyer_eef'].y,detection_wire.InValue['sawyer_eef'].angle)
		# print("abb_eef: ",detection_wire.InValue['abb_eef'].x,detection_wire.InValue['abb_eef'].y,detection_wire.InValue['abb_eef'].angle)
		# print("staubli_eef: ",detection_wire.InValue['staubli_eef'].x,detection_wire.InValue['staubli_eef'].y,detection_wire.InValue['staubli_eef'].angle)