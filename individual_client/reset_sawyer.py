from RobotRaconteur.Client import *
import time
import numpy as np

url='rr+tcp://[fe80::a2c:1efa:1c07:f043]:58654?service=sawyer'
c = RRN.ConnectService(url)
#c = RRN.ConnectService('rr+local:///?nodename=sawyer_robot&service=sawyer')

robot_info = c.robot_info
print(robot_info)

#c.reset_errors()

c.disable()

time.sleep(2)

c.enable()

#time.sleep(10)

print(hex(c.robot_state.PeekInValue()[0].robot_state_flags))

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)

halt_mode = robot_const["RobotCommandMode"]["halt"]
home_mode = robot_const["RobotCommandMode"]["homing"]


try:
    c.command_mode = halt_mode
except: pass

print(hex(c.robot_state.PeekInValue()[0].robot_state_flags))
time.sleep(0.1)
c.command_mode = home_mode
print(hex(c.robot_state.PeekInValue()[0].robot_state_flags))

homing = c.home()
try:
    while True:
        print(homing.Next())
except RR.StopIterationException: pass
