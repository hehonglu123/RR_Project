#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, yaml, traceback
from tkinter import *
from tkinter import messagebox
top=Tk()

#hardcode const here
c = RRN.ConnectService('rr+tcp://bbb2.local:58652?service=ur_robot')
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)
state_flags_enum = robot_const['RobotStateFlags']


#connection failed callback
def connect_failed(s, client_id, url, err):
	for name in robot_namelist:
		if name in url[0]:
			plug[name].config(relief="raised")
			plug[name].configure(bg='red')
			# robot_sub[name]=RRN.SubscribeService(url)

def multisplit(s, delims):
	pos = 0
	for i, c in enumerate(s):
		if c in delims:
			yield s[pos:i]
			pos = i + 1
	yield s[pos:]

	

def create_robot_yaml(name):
	yaml_dict={'robot_name':name,
	'robot_command':robot_command[name].get(),
	'height':float(height[name].get()),
	'home':list(map(float,filter(None,multisplit(home[name].get(),',')))),
	'calibration_speed':float(calibration_speed[name].get()),
	'calibration_start':list(map(float,filter(None,multisplit(calibration_start[name].get(),',')))),
	'obj_namelists':list(filter(None,multisplit(obj_namelists[name].get(),','))),
	'pick_height':float(pick_height[name].get()),
	'place_height':float(place_height[name].get()),
	'tag_position':float(tag_position[name].get()),
	'url':url[name].get()}
	with open(r'client_'+name+'.yaml', 'w') as file:
		yaml.dump(yaml_dict, file)

	return

def plug_robot(name):

	if plug[name].config('relief')[-1] == 'sunken':
		plug[name].config(relief="raised")
		plug[name].configure(bg='red')
	else:
		plug[name].config(relief="sunken")
		plug[name].configure(bg='green')
	return

def calibrate_robot(name):
	if plug[name].config('relief')[-1] == 'sunken':
		os.system("python3 ../calibration/calibration_auto.py --robot-name="+name)
	else:
		messagebox.showwarning(title=None, message='plug in '+name+' first!')
	return


robot_namelist=['sawyer','ur','abb']
#GUI field element dict
robot_name={}
robot_command={}
height={}
home={}
calibration_speed={}
calibration_start={}
obj_namelists={}
pick_height={}
place_height={}
tag_position={}
url={}
#GUI button element dict
create={}
plug={}
calibrate={}
#RR robot dict
robot_sub={}
state_w={}
flags_text={}
label={}

for i in range(len(robot_namelist)):
	Label(top, text="Robot Name").grid(row=0,column=2*i)
	Label(top, text="Robot Command").grid(row=1,column=2*i)
	Label(top, text="Robot Height").grid(row=2,column=2*i)
	Label(top, text="Home Position").grid(row=3,column=2*i)
	Label(top, text="Calibration Speed").grid(row=4,column=2*i)
	Label(top, text="Calibration Start").grid(row=5,column=2*i)
	Label(top, text="Object Names").grid(row=6,column=2*i)
	Label(top, text="Pick Height").grid(row=7,column=2*i)
	Label(top, text="Place Height").grid(row=8,column=2*i)
	Label(top, text="Tag Position").grid(row=9,column=2*i)
	Label(top, text="Service URL").grid(row=10,column=2*i)

	robot_name[robot_namelist[i]] = Entry(top)
	robot_command[robot_namelist[i]] = Entry(top)
	height[robot_namelist[i]] = Entry(top)
	home[robot_namelist[i]] = Entry(top)
	calibration_speed[robot_namelist[i]] = Entry(top)
	calibration_start[robot_namelist[i]] = Entry(top)
	obj_namelists[robot_namelist[i]] = Entry(top)
	pick_height[robot_namelist[i]] = Entry(top)
	place_height[robot_namelist[i]] = Entry(top)
	tag_position[robot_namelist[i]] = Entry(top)
	url[robot_namelist[i]] = Entry(top)

	robot_name[robot_namelist[i]].grid(row=0, column=2*i+1)
	robot_command[robot_namelist[i]].grid(row=1, column=2*i+1)
	height[robot_namelist[i]].grid(row=2, column=2*i+1)
	home[robot_namelist[i]].grid(row=3, column=2*i+1)
	calibration_speed[robot_namelist[i]].grid(row=4, column=2*i+1)
	calibration_start[robot_namelist[i]].grid(row=5, column=2*i+1)
	obj_namelists[robot_namelist[i]].grid(row=6, column=2*i+1)
	pick_height[robot_namelist[i]].grid(row=7, column=2*i+1)
	place_height[robot_namelist[i]].grid(row=8, column=2*i+1)
	tag_position[robot_namelist[i]].grid(row=9, column=2*i+1)
	url[robot_namelist[i]].grid(row=10, column=2*i+1)

	robot_name[robot_namelist[i]].insert(0,robot_namelist[i])



robot_command['sawyer'].insert(0,'velocity_command')
height['sawyer'].insert(0,0.78)
home['sawyer'].insert(0,'-0.1,0.3,0.3')
calibration_speed['sawyer'].insert(0,'0.05')
calibration_start['sawyer'].insert(0,'0.55,-0.2,0.13')
obj_namelists['sawyer'].insert(0,'bt,sp')
pick_height['sawyer'].insert(0,0.105)
place_height['sawyer'].insert(0,0.095)
tag_position['sawyer'].insert(0,-0.05)
url['sawyer'].insert(0,'rr+tcp://[fe80::a2c:1efa:1c07:f043]:58654/?nodeid=8edf99b5-96b5-4b84-9acf-952af15f0918&service=sawyer')

robot_command['ur'].insert(0,'position_command')
height['ur'].insert(0,0.87)
home['ur'].insert(0,'-0.29,0.19,0.3')
calibration_speed['ur'].insert(0,'0.07')
calibration_start['ur'].insert(0,'-0.4,0.08,-0.12')
obj_namelists['ur'].insert(0,'tp,pf')
pick_height['ur'].insert(0,0.03)
place_height['ur'].insert(0,0.03)
tag_position['ur'].insert(0,-0.05)
url['ur'].insert(0,'rr+tcp://bbb2.local:58652?service=ur_robot')

robot_command['abb'].insert(0,'position_command')
height['abb'].insert(0,0.79)
home['abb'].insert(0,'-0.1,0.3,0.4')
calibration_speed['abb'].insert(0,'0.05')
calibration_start['abb'].insert(0,'0.55,0.08,0.2')
obj_namelists['abb'].insert(0,'bt,sp')
pick_height['abb'].insert(0,0.1)
place_height['abb'].insert(0,0.1)
tag_position['abb'].insert(0,-0.05)
url['abb'].insert(0,'rr+tcp://bbb3.local:58651?service=abb_robot')


#Button
create['sawyer']=Button(top,text='Create sawyer yaml',command=lambda: create_robot_yaml('sawyer'))
calibrate['sawyer']=Button(top,text='Calibrate sawyer',command=lambda: calibrate_robot('sawyer'))
plug['sawyer']=Button(top,text='Plug sawyer',command=lambda: plug_robot('sawyer'),bg='red')
create['ur']=Button(top,text='Create ur yaml',command=lambda: create_robot_yaml('ur'))
calibrate['ur']=Button(top,text='Calibrate ur',command=lambda: calibrate_robot('ur'))
plug['ur']=Button(top,text='Plug ur',command=lambda: plug_robot('ur'),bg='red')
create['abb']=Button(top,text='Create abb yaml',command=lambda: create_robot_yaml('abb'))
calibrate['abb']=Button(top,text='Calibrate abb',command=lambda: calibrate_robot('abb'))
plug['abb']=Button(top,text='Plug abb',command=lambda: plug_robot('abb'),bg='red')


create['sawyer'].grid(row=11,column=0)
calibrate['sawyer'].grid(row=12,column=0)
plug['sawyer'].grid(row=13,column=0)
create['ur'].grid(row=11,column=2)
calibrate['ur'].grid(row=12,column=2)
plug['ur'].grid(row=13,column=2)
create['abb'].grid(row=11,column=4)
calibrate['abb'].grid(row=12,column=4)
plug['abb'].grid(row=13,column=4)

 

##RR part
def update_label(name):
		robot_state=state_w[name].TryGetInValue()
		flags_text[name] = "Robot State Flags:\n\n"
		if robot_state[0]:
			for flag_name, flag_code in state_flags_enum.items():
				if flag_code & robot_state[1].robot_state_flags != 0:
					flags_text[name] += flag_name + "\n"
		else:
			flags_text[name] += 'service not running'
		label[name].config(text = flags_text[name])
		label[name].after(500, lambda: update_label(name))


robot_sub['sawyer']=RRN.SubscribeService(str(url['sawyer'].get()))
robot_sub['sawyer'].ClientConnectFailed+= connect_failed
state_w['sawyer'] = robot_sub['sawyer'].SubscribeWire("robot_state")
label['sawyer'] = Label(top, fg = "black", justify=LEFT)
label['sawyer'].grid(row=14,column=0)
label['sawyer'].config(text="test")
label['sawyer'].after(500,lambda: update_label('sawyer'))

robot_sub['ur']=RRN.SubscribeService(str(url['ur'].get()))
robot_sub['ur'].ClientConnectFailed+= connect_failed
state_w['ur'] = robot_sub['ur'].SubscribeWire("robot_state")
label['ur'] = Label(top, fg = "black", justify=LEFT)
label['ur'].grid(row=14,column=2)
label['ur'].config(text="test")
label['ur'].after(500,lambda: update_label('ur'))

robot_sub['abb']=RRN.SubscribeService(str(url['abb'].get()))
robot_sub['abb'].ClientConnectFailed+= connect_failed
state_w['abb'] = robot_sub['abb'].SubscribeWire("robot_state")
label['abb'] = Label(top, fg = "black", justify=LEFT)
label['abb'].grid(row=14,column=4)
label['abb'].config(text="test")
label['abb'].after(500,lambda: update_label('abb'))	



cognex_sub=RRN.SubscribeService('rr+tcp://localhost:52222/?service=cognex')
cognex.ClientConnectFailed+= connect_failed

top.mainloop()
