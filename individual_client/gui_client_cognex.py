#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, time, traceback
from tkinter import *
from tkinter import messagebox
import numpy as np
sys.path.append('../toolbox')
from autodiscovery import autodiscover

#connection failed callback
def connect_failed(s, client_id, url, err):
	print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))

#auto discovery
time.sleep(2)
url=autodiscover("edu.rpi.robotics.cognex.cognex","cognex")
if url==None:
	print("service not found")
	sys.exit(1)

cognex_sub=RRN.SubscribeService(url)
cognex_sub.ClientConnectFailed+= connect_failed
detection_wire=cognex_sub.SubscribeWire("detection_wire")
while not detection_wire.TryGetInValue()[0]:
	time.sleep(0.1)
	continue
wire_value=detection_wire.TryGetInValue()
status=dict.fromkeys(wire_value[1].keys(),[])
num_rows=int(np.sqrt(len(wire_value[1])))



top=Tk()
top.title("Cognex objects status")
def update_status(status):
	wire_value=detection_wire.TryGetInValue()
	if wire_value[0]:
		for key in wire_value[1]:
			if wire_value[1][key].detected:
				status[key].configure(bg='green')
			else:
				status[key].configure(bg='red')

	top.after(500, lambda: update_status(status))


i=0
for key in status:
	status[key]=Canvas(top, width=20, height=20,bg = 'red')
	Label(top, text=key+ " status: ").grid(row=int(i/num_rows),column=int(2*(i%num_rows)))
	status[key].grid(row=int(i/num_rows), column=int(2*(i%num_rows)+1))
	i+=1

top.after(500, lambda: update_status(status))

top.mainloop()
