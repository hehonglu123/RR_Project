#Simple example Robot Raconteur Sawyer Robot and Cognex client
#pick up and drop detected objects
from RobotRaconteur.Client import *

import numpy as np
import time,traceback,sys
sys.path.append('../toolbox')
from autodiscovery import autodiscover

#connection failed callback
def connect_failed(s, client_id, url, err):
	print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))


####################Start Service and robot setup
# url='rr+tcp://[fe80::922f:c9e6:5fe5:51d1]:52222/?nodeid=87518815-d3a3-4e33-a1be-13325da2461f&service=cognex'
#auto discovery
time.sleep(2)
url=autodiscover("edu.robotraconteur.objectrecognition.ObjectRecognitionSensor","cognex")
if url==None:
	print("service not found")
	sys.exit(1)

cognex_sub=RRN.SubscribeService(url)

detection_wire=cognex_sub.SubscribeWire("detection_wire")
time.sleep(.5)

wire_value=detection_wire.TryGetInValue()
if wire_value[0]:
	for key, value in wire_value[1].items():
		print(key+':'+str(value.detected))
		if value.detected:
			print('x: ',value.x,'y: ',value.y,'angle: ',value.angle)
else:
	print("wire value not set")