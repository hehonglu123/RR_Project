# from RobotRaconteur.Client import *
# import os
# cwd = os.getcwd()
# #register robot service definition
# directory='/home/rpi/catkin_ws/src/robotraconteur_companion/robdef/group1/'
# os.chdir(directory)
# RRN.RegisterServiceTypesFromFiles(['com.robotraconteur.robotics.robot.robdef'],True)
# os.chdir(cwd)


import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import os
from RobotRaconteur.RobotRaconteurPythonUtil import ReadServiceDefinitionFiles
cwd = os.getcwd()
directory='/home/rpi/catkin_ws/src/robotraconteur_companion/robdef/group1/'
os.chdir(directory)
d=ReadServiceDefinitionFiles(['com.robotraconteur.imaging'],True)



RRN._RegisterServiceTypes(d)
os.chdir(cwd)