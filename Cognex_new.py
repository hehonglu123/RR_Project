# -*- coding: utf-8 -*-

#Simple example Robot Raconteur Industrial Cognex service

import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import socket, threading, traceback, copy, time, signal

host = '128.113.224.154'		#IP address of PC
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
		#initialize socket connection
		self.s=socket.socket()
		self.s.bind((host, port))
		self.s.listen(5)
		self.c,addr=self.s.accept()

		#threading setting
		self._lock=threading.RLock()
		self._running=False
		
		#initialize detection obj map
		self.detection_objects={}
		self.detection_obj=RRN.NewStructure("edu.rpi.robotics.cognex.detection_obj")
		self.models=['tp','pf','sp','bt','t_f','p_f','s_f','b_f','eef']
		for name in self.models:
			self.detection_objects[name]=copy.deepcopy(self.detection_obj)
			self.detection_objects[name].name=name
		
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
					string_data = self.c.recv(1024).decode("utf-8") 
					string_data=string_data.split('{')			#find leading text
					object_list = string_data[-1].split(";")	# split different object info in string
					object_list.pop(0)

					for i in range(len(object_list)):  					# split the data from cognex and parse to RR object
						general = object_list[i].split(":")	
						name=general[0]
						if '#ERR' not in general[1]:			#if detected
							
							info = list(filter(None, multisplit(general[1], '(),=°\r\n')))
							self.detection_objects[name].x = float(info[0])/1000.
							self.detection_objects[name].y = float(info[1])/1000.
							try:
								self.detection_objects[name].angle = float(info[-1])	#incase some don't need angle
							except IndexError:
								pass
							self.detection_objects[name].detected=True
						else:
							self.detection_objects[name].detected=False

					#pass to RR wire
					self.detection_wire.OutValue=self.detection_objects
				except:
					traceback.print_exc()



with RR.ServerNodeSetup("cognex_Service", 52222) as node_setup:
	
	RRN.RegisterServiceTypeFromFile("robdef/edu.rpi.robotics.cognex")
	cognex_inst=create_impl()
	cognex_inst.start()
	time.sleep(1)
	for key, value in cognex_inst.detection_objects.items():
		print('object name:', value.name)
		print('object detect:', value.detected)
		print('object x:', value.x)
		print('object y:', value.y)
		print('object angle:', value.angle)


	#add authentication for RR connections
	#password: cats111!
	# authdata="cats be7af03a538bf30343a501cb1c8237a0 objectlock"
	# p=RR.PasswordFileUserAuthenticator(authdata)
	# policies={"requirevaliduser" : "true"}
	# security=RR.ServiceSecurityPolicy(p,policies)
	# RRN.RegisterService("Cognex", "edu.rpi.robotics.cognex.cognex", cognex_inst,security)

	RRN.RegisterService("cognex", "edu.rpi.robotics.cognex.cognex", cognex_inst)

	#Add allowed origin for Web
	node_setup.tcp_transport.AddWebSocketAllowedOrigin("http://localhost")
	node_setup.tcp_transport.AddWebSocketAllowedOrigin("http://localhost:8000")
	node_setup.tcp_transport.AddWebSocketAllowedOrigin("https://johnwason.github.io")
	node_setup.tcp_transport.AddWebSocketAllowedOrigin("https://johnwason.github.io:443")

	signal.sigwait([signal.SIGTERM,signal.SIGINT])
	cognex_inst.close()
	cognex_inst.s.close()
