#Simple example Robot Raconteur Sawyer Robot and Cognex client
#pick up and drop detected objects
from RobotRaconteur.Client import *

import numpy as np
import time
import traceback
import sys

#connection failed callback
def connect_failed(s, client_id, url, err):
	print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))


####################Start Service and robot setup
# url='rr+tcp://localhost:52222/?service=cognex'
#auto discovery
time.sleep(1)
res=RRN.FindServiceByType("edu.rpi.robotics.cognex.cognex",
["rr+local","rr+tcp","rrs+tcp"])
try:
	url=res[0].ConnectionURL
except:
	print('service not found')
	sys.exit()


sub=RRN.SubscribeService(url)

obj = sub.GetDefaultClientWait(1)
detection_wire=sub.SubscribeWire("detection_wire")

time.sleep(.5)
wire_value=detection_wire.TryGetInValue()
if wire_value[0]:
	for key, value in wire_value[1].items():
		print(key+':'+str(value.detected))
		if value.detected:
			print('x: ',value.x,'y: ',value.y,'angle: ',value.angle)
else:
	print("wire value not set")