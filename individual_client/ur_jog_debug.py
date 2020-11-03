from RobotRaconteur.Client import *
import sys, time, yaml, traceback
import numpy as np
sys.path.append('../')
from vel_emulate_sub import EmulatedVelocityControl


robot_name='ur'
url='rr+tcp://[fe80::76d6:e60f:27f6:1e3e]:58653/?nodeid=55ade648-a8c2-4775-a7ec-645acea83525&service=robot'
robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = jog_mode


##robot wire
cmd_w = robot_sub.SubscribeWire("position_command")
state_w = robot_sub.SubscribeWire("robot_state")

vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w, 0.01)
time.sleep(1)

home=np.array([-0.47422068, -1.71062362,  1.87470162, -1.73891947, -1.57458153, -1.74867896])
qd=np.array([ 0.10082249, -1.61621327,  1.76972166, -1.75459159, -1.57169486, -1.01491468])
qd2=np.array([ 0.12337597, -1.35698388,  2.14508136, -2.34484581, -1.57291774, -1.28153818])


#go home first with jog
robot.jog_freespace(home, np.ones(6), True)


#switch to position mode
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = position_mode
#enable velocity mode
vel_ctrl.enable_velocity_mode()
#cal qdot
qdot=(qd-vel_ctrl.joint_position())/1.

now=time.time()
while time.time()-now<1:
	vel_ctrl.set_velocity_command(qdot)
	time.sleep(0.01)
#disable 
vel_ctrl.set_velocity_command([0,0,0,0,0,0])
vel_ctrl.disable_velocity_mode() 
#switch to jog mode again
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = jog_mode
robot.jog_freespace(qd2, np.ones(6), True)
