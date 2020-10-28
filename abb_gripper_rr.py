from relay_lib_seeed import *
import time
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


with RR.ServerNodeSetup("abb_gripper",11222) as node_setup:
    RRN.RegisterServiceTypeFromFile("edu.rpi.robotics.abbgripper")
    gripper_inst=create_gripper()
    RRN.RegisterService("abb_gripper","edu.rpi.robotics.abbgripper.gripper",gripper_inst)

    input("Press enter to quit")
