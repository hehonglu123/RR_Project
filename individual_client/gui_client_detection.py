#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, time, traceback, argparse
from tkinter import *
from tkinter import messagebox
import numpy as np
sys.path.append('../toolbox')
from autodiscovery import autodiscover

#connection failed callback
def connect_failed(s, client_id, url, err):
	print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))

#Accept the names of the webcams and the nodename from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--service-name",type=str,help="List of camera names separated with commas")
args, _ = parser.parse_known_args()


#auto discovery
time.sleep(2)
url=autodiscover("edu.robotraconteur.cognexsensor.CognexSensor",args.service_name)
print(url)
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
centroids_label=dict.fromkeys(wire_value[1].keys(),[])
# num_rows=int(np.sqrt(len(wire_value[1])))

num_rows=1

top=Tk()
top.title("Cognex objects status")
def update_status(status):
	wire_value=detection_wire.TryGetInValue()
	if wire_value[0]:
		for key in wire_value[1]:
			if wire_value[1][key].detected:
				status[key].configure(bg='green')
				centroids_label[key].config(text = 'x: '+"{:.3f}".format(wire_value[1][key].x)+', y: '+"{:.3f}".format(wire_value[1][key].y)+', angle: '+"{:.1f}".format(wire_value[1][key].angle))
			else:
				status[key].configure(bg='red')
				centroids_label[key].config(text ='x: 0, y: 0')

	top.after(500, lambda: update_status(status))


i=0
for key in status:
	status[key]=Canvas(top, width=20, height=20,bg = 'red')
	centroids_label[key]=Label(top, fg = "black", justify=LEFT)
	Label(top, text=key).grid(row=int(i/num_rows),column=int(3*(i%num_rows)))
	status[key].grid(row=int(i/num_rows), column=int(3*(i%num_rows)+1))
	centroids_label[key].grid(row=int(i/num_rows), column=int(3*(i%num_rows)+2))
	i+=1

top.after(500, lambda: update_status(status))

top.mainloop()
