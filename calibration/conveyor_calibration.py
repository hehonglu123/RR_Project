import numpy as np
from RobotRaconteur.Client import *
import sys, time, yaml

#Connect to Cognex

url='rr+tcp://localhost:52222/?service=cognex'

sub=RRN.SubscribeService(url)
while True:
	try:
		obj = sub.GetDefaultClient()
		detection_wire=sub.SubscribeWire("detection_wire")
		break
	except RR.ConnectionException:
		time.sleep(0.1)