#Simple example Robot Raconteur robot rolling client
from RobotRaconteur.Client import *

####################Start Service and robot setup
url='rr+tcp://localhost:25522?service=Environment'

obj = RRN.ConnectService(url)
obj.roll("abb",3.,3.,0.,1.57)
