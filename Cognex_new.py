# -*- coding: utf-8 -*-

#Simple example Robot Raconteur Industrial Cognex service


import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import struct
import socket
import thread
import threading
import traceback

host = '128.113.224.144'		#IP address of PC
port = 3000

def multisplit(s, delims):
	pos = 0
	for i, c in enumerate(s):
		if c in delims:
			yield s[pos:i]
			pos = i + 1
	yield s[pos:]


class create_impl(object):
	def __init__(self):
		self._lock=threading.RLock()
		self._running=False

	def start(self):
		self._running=True
		self._camera = threading.Thread(target=self.object_update)
		self._camera.daemon = True
		self._camera.start()
	def close(self):
		self._running = False
		self._camera.join()

	def object_update(self):
		while self._running:
			with self._lock:
				try:
					global c
					temp_objects=[]
					string_data = c.recv(1024)
					string_data=string_data.split('{')			#find leading text
					object_list = string_data[-1].split(";")	# split different object info in string
					object_list.pop(0)
					number=len(object_list)
					for i in range(number):  					# split the data from smartcam and parse to RR object
						general = object_list[i].split(":")	
						if '#ERR' not in general[1]:			#add detected object to object list
							temp_object=RR.RobotRaconteurNode.s.NewStructure("edu.rpi.robotics.smartcam.obj")
							temp_object.name = general[0]
							info = list(filter(None, multisplit(general[1], '(),=°\r\n')))
							temp_object.detect = 1
							temp_object.x = float(info[0])
							temp_object.y = float(info[1])
							temp_object.angle = float(info[2])
							temp_object.obj_pass=1
							temp_objects.append(temp_object)
					self.number = len(object_list)  			# determine number of objects
					self.objects=temp_objects
				except:
					traceback.print_exc()


	def update(self):
		global c
		self.objects=[]								#clean the list

		string_data = c.recv(2048)
		# print(string_data)
		string_data=string_data.split('{')			#find leading text
		if (len(string_data)>2):
			object_list = string_data[-2].split(";")# split different object info in string
		else:
			object_list = string_data[-1].split(";")# split different object info in string
													#clean up socket buffer

		object_list.pop(0)
		self.number = len(object_list)  			# determine number of objects
		for i in range(self.number):  				# split the data from smartcam and parse to RR object

			general = object_list[i].split(":")	
			if '#ERR' not in general[1]:			#add detected object to object list
				self.objects.append(RR.RobotRaconteurNode.s.NewStructure("edu.rpi.robotics.smartcam.obj"))
				self.objects[-1].name = general[0]
				info = list(filter(None, multisplit(general[1], '(),=°\r\n')))
				self.objects[-1].detect = 1
				self.objects[-1].x = float(info[0])
				self.objects[-1].y = float(info[1])
				self.objects[-1].angle = float(info[2])
				self.objects[-1].obj_pass=1



with RR.ServerNodeSetup("SmartCam_Service", 52222) as node_setup:
	global c
	s=socket.socket()
	s.bind((host, port))
	s.listen(5)							#connect to smartcam client
	c,addr=s.accept()

	RRN.RegisterServiceTypeFromFile("robdef/edu.rpi.robotics.smartcam")
	inst=create_impl()
	inst.start()
	inst.update()
	for i in range(len(inst.objects)):
		print('object name:', inst.objects[i].name)
		print('object detect:', inst.objects[i].detect)
		print('object x:', inst.objects[i].x)
		print('object y:', inst.objects[i].y)
		print('object angle:', inst.objects[i].angle)


	#add authentication for RR connections
	#password: cats111!
	authdata="cats be7af03a538bf30343a501cb1c8237a0 objectlock"
	p=RR.PasswordFileUserAuthenticator(authdata)
	policies={"requirevaliduser" : "true"}
	security=RR.ServiceSecurityPolicy(p,policies)
	RRN.RegisterService("SmartCam", "edu.rpi.robotics.smartcam.smartcam", inst,security)

	#Add allowed origin for Web
	node_setup.tcp_transport.AddWebSocketAllowedOrigin("http://localhost")
	node_setup.tcp_transport.AddWebSocketAllowedOrigin("http://localhost:8000")
	node_setup.tcp_transport.AddWebSocketAllowedOrigin("https://johnwason.github.io")
	node_setup.tcp_transport.AddWebSocketAllowedOrigin("https://johnwason.github.io:443")

	input("Press enter to quit")
	inst.close()
	s.close()
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
