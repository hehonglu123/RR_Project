# -*- coding: utf-8 -*-

#Simple example Robot Raconteur Industrial Cognex service

import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np
import socket, threading, traceback, copy, time, os, signal

host = '169.254.235.13'		#IP address of PC
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
		
		#initialize objrecog structures
		self._object_recognition_sensor_data=None
		self.object_recognition_sensor_data_data=RRN.NewStructure("edu.robotraconteur.objectrecognition.ObjectRecognitionSensorData")
		self.object_recognition_sensor_data_data.recognized_objects=RRN.NewStructure("edu.robotraconteur.objectrecognition.RecognizedObjects") 	#recognized_objects

		self.recognized_object=RRN.NewStructure("edu.robotraconteur.objectrecognition.RecognizedObject")
		self.recognized_object.recognized_object=RRN.NewStructure("com.robotraconteur.identifier.Identifier")					#name,uuid
		uuid_dtype=RRN.GetNamedArrayDType("com.robotraconteur.uuid.UUID")
		self.recognized_object.recognized_object.uuid=np.zeros((1,),dtype=uuid_dtype)
		self.recognized_object.pose=RRN.NewStructure("com.robotraconteur.geometry.NamedPoseWithCovariance")
		self.recognized_object.pose.pose=RRN.NewStructure("com.robotraconteur.geometry.NamedPose")
		pose_dtype=RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose")
		self.recognized_object.pose.pose.pose=np.zeros((1,),dtype=pose_dtype)

		#initialize detection obj map
		self.detection_objects={}
		self.detection_obj=RRN.NewStructure("edu.robotraconteur.objectrecognition.detection_obj")
		self.models=['tp','pf','sp','bt','t_f','p_f','s_f','b_f','eef']
		for name in self.models:
			self.detection_objects[name]=copy.deepcopy(self.detection_obj)
			self.detection_objects[name].name=name

		
	#object_recognition_sensor_data pipe member property getter and setter
	@property
	def object_recognition_sensor_data(self):
		return self._object_recognition_sensor_data
	@object_recognition_sensor_data.setter
	def object_recognition_sensor_data(self,data):
		self._object_recognition_sensor_data=data
		#Create the PipeBroadcaster and set backlog to 3 so packets
		#will be dropped if the transport is overloaded
		self._object_recognition_sensor_data_broadcaster=RR.PipeBroadcaster(data,1)
		
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

					#clear list
					self.object_recognition_sensor_data_data.recognized_objects.recognized_objects=[]

					for i in range(len(object_list)):  					# split the data from cognex and parse to RR object
						general = object_list[i].split(":")	
						name=general[0]
						if '#ERR' not in general[1]:			#if detected
							
							info = list(filter(None, multisplit(general[1], '(),=°\r\n')))
							#standard type
							self.recognized_object.recognized_object.name=name
							self.recognized_object.pose.pose.pose['position']['x']= float(info[0])/1000.
							self.recognized_object.pose.pose.pose['position']['y']= float(info[1])/1000.
							#my type
							self.detection_objects[name].x = float(info[0])/1000.
							self.detection_objects[name].y = float(info[1])/1000.
							try:
								self.recognized_object.pose.pose.pose['orientation']['z'] = float(info[-1])	#incase some don't need angle
								self.detection_objects[name].angle = float(info[-1])
							except IndexError:
								pass
							#append to list
							self.object_recognition_sensor_data_data.recognized_objects.recognized_objects.append(copy.deepcopy(self.recognized_object))
							#my type
							self.detection_objects[name].detected=True
						else:
							self.detection_objects[name].detected=False

					#pass to RR wire
					self.detection_wire.OutValue=self.detection_objects
					#pass to RR pipe
					self._object_recognition_sensor_data_broadcaster.AsyncSendPacket(self.object_recognition_sensor_data_data,lambda: None)  
				except:
					traceback.print_exc()



with RR.ServerNodeSetup("cognex_Service", 52222) as node_setup:
	#register objdet robdef
	os.chdir('/home/ubuntu/catkin_ws/src/robotraconteur_companion/robdef/group1')
	RRN.RegisterServiceTypesFromFiles(['edu.robotraconteur.objectrecognition.robdef'],True) 

	cognex_inst=create_impl()
	cognex_inst.start()
	time.sleep(1)
	for key, value in cognex_inst.detection_objects.items():
		print('object name:', value.name)
		print('object detect:', value.detected)
		print('object x:', value.x)
		print('object y:', value.y)
		print('object angle:', value.angle)

	RRN.RegisterService("cognex", "edu.robotraconteur.objectrecognition.ObjectRecognitionSensor", cognex_inst)

	print("press ctrl+c to quit")
	signal.sigwait([signal.SIGTERM,signal.SIGINT])
	cognex_inst.close()
	cognex_inst.s.close()
