# -*- coding: utf-8 -*-

#Simple example Robot Raconteur Industrial Cognex service

import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np
import socket, threading, traceback, copy, time, os

host = '128.113.224.154'		#IP address of PC
port = 3000
#register objdet robdef
os.chdir('/home/iamnotedible/catkin_ws/src/robotraconteur_companion/robdef/group1')
RRN.RegisterServiceTypesFromFiles(['com.robotraconteur.objectrecognition.robdef'],True) 


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
		self._object_recognition_sensor_data=None
		self.object_recognition_sensor_data_data=RRN.NewStructure("com.robotraconteur.objectrecognition.ObjectRecognitionSensorData")
		self.object_recognition_sensor_data_data.recognized_objects=RRN.NewStructure("com.robotraconteur.objectrecognition.RecognizedObjects") 	#recognized_objects


		self.recognized_object=RRN.NewStructure("com.robotraconteur.objectrecognition.RecognizedObject")
		self.recognized_object.recognized_object=RRN.NewStructure("com.robotraconteur.identifier.Identifier")					#name,uuid
		self.recognized_object.pose=RRN.NewStructure("com.robotraconteur.geometry.NamedPoseWithCovariance")
		self.recognized_object.pose.pose=RRN.NewStructure("com.robotraconteur.geometry.NamedPose")


		pose_dtype=RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose")
		self.recognized_object.pose.pose.pose=np.zeros((1,),dtype=pose_dtype)


		
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

							self.recognized_object.recognized_object.name=name
							self.recognized_object.pose.pose.pose['position']['x']= float(info[0])/1000.
							self.recognized_object.pose.pose.pose['position']['y']= float(info[1])/1000.
							try:
								self.recognized_object.pose.pose.pose['orientation']['z'] = float(info[-1])	#incase some don't need angle
							except IndexError:
								pass

							self.object_recognition_sensor_data_data.recognized_objects.recognized_objects.append(copy.deepcopy(self.recognized_object))

					#pass to RR pipe
					self._turtlechange_broadcaster.AsyncSendPacket(self.object_recognition_sensor_data_data,lambda: None)  
				except:
					traceback.print_exc()



with RR.ServerNodeSetup("cognex_Service", 52222) as node_setup:


	cognex_inst=create_impl()
	cognex_inst.start()
	time.sleep(1)


	RRN.RegisterService("cognex", "com.robotraconteur.objectrecognition.ObjectRecognitionSensor", cognex_inst)



	input("Press enter to quit")
	cognex_inst.close()
	cognex_inst.s.close()
