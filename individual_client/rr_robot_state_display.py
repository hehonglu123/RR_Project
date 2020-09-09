from RobotRaconteur.Client import *
import tkinter as tk
import time, sys, yaml
import numpy as np

#read in robot name and import proper libraries
if (sys.version_info > (3, 0)):
    robot_name=input('robot name: ')
else:
    robot=raw_input('robot name: ')
with open(r'../client_yaml/client_'+robot_name+'.yaml') as file:
    robot_yaml = yaml.load(file, Loader=yaml.FullLoader)
url=robot_yaml['url']

c = RRN.ConnectService(url)

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)
state_flags_enum = robot_const['RobotStateFlags']

root = tk.Tk()
root.title = "Robot State"

label = tk.Label(root, fg = "black", justify=tk.LEFT)
label.pack()
label.config(text="test")

def update_label():

    robot_state = c.robot_state.PeekInValue()[0]

    flags_text = "Robot State Flags:\n\n"

    for flag_name, flag_code in state_flags_enum.items():
        if flag_code & robot_state.robot_state_flags != 0:
            flags_text += flag_name + "\n"

    

    joint_text = "Robot Joint Positions:\n\n"
    for j in robot_state.joint_position:
        joint_text += "%.2f\n" % np.rad2deg(j)

    label.config(text = flags_text + "\n\n" + joint_text)

    label.after(250, update_label)

    

label.after(250,update_label)
root.mainloop()
