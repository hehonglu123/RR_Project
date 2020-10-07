#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback
from tkinter import *
from tkinter import messagebox
from qpsolvers import solve_qp
import numpy as np
sys.path.append('../')
from vel_emulate_sub import EmulatedVelocityControl

sys.path.append('../toolbox')
from general_robotics_toolbox import *     

def move(n, robot_def,vel_ctrl,vd):
	#enable velocity mode
    vel_ctrl.enable_velocity_mode()
    Kq=.01*np.eye(n)    #small value to make sure positive definite

    J=robotjacobian(robot_def,vel_ctrl.joint_position())        #calculate current Jacobian
    Jp=J[3:,:]
    H=np.dot(np.transpose(Jp),Jp)+Kq 
    H=(H+np.transpose(H))/2
    f=-np.dot(np.transpose(Jp),np.array(vd))
    qdot=solve_qp(H, f)
    vel_ctrl.set_velocity_command(qdot)
    vel_ctrl.set_velocity_command(np.zeros((n,)))
    vel_ctrl.disable_velocity_mode()
    return


#Accept the names of the webcams and the nodename from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--robot-name",type=str,help="List of camera names separated with commas")
args, _ = parser.parse_known_args()
robot_name=args.robot_name
#auto discovery
time.sleep(2)
res=RRN.FindServiceByType("com.robotraconteur.robotics.robot.Robot",
["rr+local","rr+tcp","rrs+tcp"])
url=None
for serviceinfo2 in res:
	if robot_name in serviceinfo2.Name:
		url=serviceinfo2.ConnectionURL
		break
if url==None:
	print('service not found')
	sys.exit()


num_joints=len(robot.robot_info.joint_info)
P=np.array(robot.robot_info.chains[0].P.tolist())
length=np.linalg.norm(P[1])+np.linalg.norm(P[2])+np.linalg.norm(P[3])
H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))

robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)
state_w = robot_sub.SubscribeWire("robot_state")

##########Initialize robot constants
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
robot.command_mode = halt_mode
cmd_w = robot_sub.SubscribeWire("position_command")

vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w)


#parameter setup
n= len(robot.robot_info.joint_info)

top=Tk()
##RR part
def update_label():
		robot_state=state_w.TryGetInValue()
		flags_text = "Robot State Flags:\n\n"
		if robot_state[0]:
			for flag_name, flag_code in state_flags_enum.items():
				if flag_code & robot_state[1].robot_state_flags != 0:
					flags_text += flag_name + "\n"
		else:
			flags_text += 'service not running'
		label[name].config(text = flags_text)
		label[name].after(250, lambda: update_label())

top.title = "Robot State"

label = Label(top, fg = "black", justify=LEFT)
label.pack()
label.after(250,update_label)


left=Button(top,text='left',command=lambda: move(num_joints,robot,robot_def,[0,.1,0]))
right=Button(top,text='right',command=lambda: move(num_joints,robot,robot_def,[0,-.1,0]))
forward=Button(top,text='forward',command=lambda: move(num_joints,robot,robot_def,[.1,0,0]))
backward=Button(top,text='backward',command=lambda: move(num_joints,robot,robot_def,[-.1,0,0]))
up=Button(top,text='up',command=lambda: move(num_joints,robot,robot_def,[0,0,.1]))
down=Button(top,text='down',command=lambda: move(num_joints,robot,robot_def,[0,0,-.1]))

left.pack()
right.pack()
forward.pack()
backward.pack()
up.pack()
down.pack()



top.mainloop()