from relay_lib_seeed import *
import time, os, signal
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s


DEVICE_ADDRESS= 0x20

class create_gripper(object):
    def __init__(self):
        self.port=1
    def open(self):
        relay_off(self.port)
    def close(self):
        relay_on(self.port)

        
with RR.ServerNodeSetup("abb_gripper",50500) as node_setup:
    os.chdir('/home/duckiebot/robotraconteur_standard_robdef/group1')
    RRN.RegisterServiceTypesFromFiles(['com.robotraconteur.robotics.tool.robdef'],True) 
    gripper_inst=create_gripper()
    RRN.RegisterService("tool","com.robotraconteur.robotics.tool.Tool",gripper_inst)

    print("Press ctrl+c to quit")
    signal.sigwait([signal.SIGTERM,signal.SIGINT])
