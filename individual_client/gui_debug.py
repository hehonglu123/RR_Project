from RobotRaconteur.Client import *

import numpy as np
import time
import traceback
import sys

#connection failed callback
def connect_failed(s, client_id, url, err):
	print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))



# sawyer_sub=RRN.SubscribeService('rr+tcp://128.113.224.23:58654/?nodeid=8edf99b5-96b5-4b84-9acf-952af15f0918&service=sawyer')
# sawyer_sub.ClientConnectFailed+= connect_failed


# ur_sub=RRN.SubscribeService('rr+tcp://128.113.224.83:58652?service=ur_robot')
# ur_sub.ClientConnectFailed+= connect_failed


abb_sub=RRN.SubscribeService('rr+tcp://bbb3.local:58651?service=abb_robot')
abb_sub.ClientConnectFailed+= connect_failed

# cognex_sub=RRN.SubscribeService('rr+tcp://[fe80::922f:c9e6:5fe5:51d1]:52222/?nodeid=87518815-d3a3-4e33-a1be-13325da2461f&service=cognex')
# cognex_sub.ClientConnectFailed+= connect_failed
RRN.ConnectService('rr+tcp://128.113.224.69:52222/?nodeid=87518815-d3a3-4e33-a1be-13325da2461f&service=cognex')


input('press enter to quit')
