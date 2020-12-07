import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np
import socket, threading, traceback, copy, time, os, signal
import cv2
import numpy as np


class create_impl(object):
	def __init__(self,view):
		self.cam = cv2.VideoCapture(0)
		self.view=view
		self.processed_img=None
		#threading setting
		self._lock=threading.RLock()
		self._running=False

	def start(self):
		self._running=True
		self._camera = threading.Thread(target=self.detection)
		self._camera.daemon = True
		self._camera.start()
	def close(self):
		self._running = False
		self._camera.join()


	def detection(self):
		ret, img = self.read()
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		edges = cv2.Canny(gray,50,150,apertureSize = 3)

		# sobely = cv2.Sobel(gray,cv2.CV_64F,0,1,ksize=5)  # y
		# sobelx = cv2.Sobel(gray,cv2.CV_64F,1,0,ksize=5)  # x
		# alpha=-np.pi/4
		# edges=np.sqrt((sobely*np.sin(alpha))**2 + (sobelx*np.cos(alpha))**2)

		#filter region of interest
		for y in range(len(edges)):
			for x in range(len(edges[0])):
				#filter out of conveyor belt
				if 0.25*float(x)+y<300 or 0.7364*float(x)+y>585 or y<200:
					edges[y][x]=0.

		# edges = cv2.threshold(edges, 200, 255, cv2.THRESH_BINARY)[1]
		# edges=edges.astype(np.uint8)
		# retval, labels, stats, centroids=cv2.connectedComponentsWithStats(edges) #run CCC on the filtered image
		# index=np.argmax(stats[1:,4])
		# #get largest connected part
		# edges=labels==(index+1)
		# edges=edges.astype(np.uint8)*255

		lines = cv2.HoughLinesP(edges,rho=1,theta=np.pi/180,threshold=60,minLineLength=100,maxLineGap=10)
		for line in lines:
			x1,y1,x2,y2 = line[0]
			slope=(y1-y2)/(x1-x2)
			if slope>0 and slope<1:
				cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)

		self.processed_img=copy.deepcopy(img)

with RR.ServerNodeSetup("box_Service", 53333) as node_setup:

	parser = argparse.ArgumentParser(description="box edge detection service")
	parser.add_argument("--view",type=bool,default=False)
	args, _ = parser.parse_known_args()

	RRN.RegisterServiceTypesFromFiles(['service edu.rpi.robotics.box.robdef'],True) 

	box_inst=create_impl(args.view)
	box_inst.start()
	while args.view:
		if (not box_inst.processed_img is None):
				cv2.imshow("Detection Image",box_inst.processed_img)
			if cv2.waitKey(50)!=-1:
				break
	cv2.destroyAllWindows()


	box_inst.close()

if __name__ == '__main__':
	
	main()
