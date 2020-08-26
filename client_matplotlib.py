################Real time object $ Robot display in matplotlib

import RobotRaconteur as RR
from RobotRaconteur.Client import *
import numpy as np
from general_robotics_toolbox import *
from inv_kin import lab_invk
####################Start Service
inst=RRN.ConnectService('rr+tcp://128.113.224.144:52222/?service=SmartCam',"cats",{"password":RR.RobotRaconteurVarValue("cats111!","string")},None,None)
Sawyer=RRN.ConnectService('tcp://128.113.224.144:8884/SawyerJointServer/Sawyer',"cats",{"password":RR.RobotRaconteurVarValue("cats111!","string")},None,None)
UR=RRN.ConnectService('tcp://128.113.224.144:2355/URConnection/Universal_Robot',"cats",{"password":RR.RobotRaconteurVarValue("cats111!","string")},None,None)
c_host=RRN.ConnectService('rr+tcp://localhost:2366?service=Webcam',"cats",{"password":RR.RobotRaconteurVarValue("cats111!","string")},None,None)

import datetime as dt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

ex=np.array([[1],[0],[0]])
ey=np.array([[0],[1],[0]])
ez=np.array([[0],[0],[1]])
H=np.concatenate((ez,-ey,-ey,-ey,-ez,-ey),axis=1)
p0=np.array([[0],[0],[0]])
p1=np.array([[0],[0],[0.089159]])
p2=np.array([[-0.425],[0],[0]])
p3=np.array([[-0.39225],[0],[0]])
p4=np.array([[0],[-0.10915],[0]])
p5=np.array([[0],[0],[-0.09465]])
p6=np.array([[0],[-0.0823],[0]])
P=np.concatenate((p0,p1,p2,p3,p4,p5,p6),axis=1)
joint_type=np.zeros(6)
UR_def=Robot(H,P,joint_type)


def H_inv(H):							#inverse the homogeneous transformation matrix
	R=H[:2,:2]
	R=np.transpose(R)
	d=np.transpose(np.array([H[:2,2]]))
	d=-np.dot(R,d)
	H=np.concatenate((np.concatenate((R,d),axis=1),np.array([[0,0,1]])),axis=0)
	return H


H_Sawyer=np.array([[0.99942127,0.03401641,0.01130381],[-0.03401641,0.99942127,0.05279206],[0,0,1]])
H_UR=np.array([[0.04223891, -0.99910754,  0.00630481],[0.99910754,  0.04223891, -1.3026918],[0,0,1]])

H_S_C=H_inv(H_Sawyer)
H_UR_C=H_inv(H_UR)					#map robot coordinate to camera coordinate

# Create figure for plotting
fig, (ax, ax1) = plt.subplots(2)




def animate(i):
	global Sawyer, UR
	#get UR and Sawyer state info
	sawyer_state = Sawyer.robot_state.PeekInValue()
	pose_Sawyer=sawyer_state[0].kin_chain_tcp
	q_Sawyer=sawyer_state[0].joint_position
	ur_state = UR.robot_state.PeekInValue()
	q_UR=ur_state[0].joint_position						
	pose_UR=ur_state[0].kin_chain_tcp


	xs = []
	ys = []

	# Add x and y to lists
	for obj in inst.objects:
		xs.append(obj.x/1000.)
		ys.append(obj.y/1000.)

	# Draw x and y lists
	ax.clear()
	ax.plot(xs, ys,'ro')
	pose_Sawyer_C=np.dot(H_S_C,np.array([[pose_Sawyer[0]['position']['x']],[pose_Sawyer[0]['position']['y']],[1]]))
	ax.plot(pose_Sawyer_C[0],pose_Sawyer_C[1],'ro',color='blue')
	pose_UR_C=np.dot(H_UR_C,np.array([[pose_UR[0]['position']['x']],[pose_UR[0]['position']['y']],[1]]))
	print(pose_UR)

	# pose_UR=fwdkin(UR_def,q_UR).p
	# pose_UR_C=np.dot(H_UR_C,np.array([[pose_UR[0]],[pose_UR[1]],[1]]))

	ax.plot(pose_UR_C[0],pose_UR_C[1],'ro',color='blue')
	ax.set(xlim=(-1, 2), ylim=(-1, 2))
	props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)

	text_UR = '\n'.join((
	r'$q1=%.2f$' % (q_UR[0], ),
	r'$q2=%.2f$' % (q_UR[1], ),
	r'$q3=%.2f$' % (q_UR[2], ),
	r'$q4=%.2f$' % (q_UR[3], ),
	r'$q5=%.2f$' % (q_UR[4], ),
	r'$q6=%.2f$' % (q_UR[5], )))

	ax.text(0.75, 0.95, text_UR, transform=ax.transAxes, fontsize=14,
			verticalalignment='top', bbox=props)
	text_Sawyer = '\n'.join((
	r'$q1=%.2f$' % (q_Sawyer[0], ),
	r'$q2=%.2f$' % (q_Sawyer[1], ),
	r'$q3=%.2f$' % (q_Sawyer[2], ),
	r'$q4=%.2f$' % (q_Sawyer[3], ),
	r'$q5=%.2f$' % (q_Sawyer[4], ),
	r'$q6=%.2f$' % (q_Sawyer[5], ),
	r'$q7=%.2f$' % (q_Sawyer[6], )))


	# place a text box in upper left in axes coords
	ax.text(0.02, 0.1, text_Sawyer, transform=ax.transAxes, fontsize=14,
			bbox=props)

	text_Sawyer_end = '\n'.join((
	r'$x=%.2f$' % (pose_Sawyer_C[0], ),
	r'$y=%.2f$' % (pose_Sawyer_C[1], )))
	ax.text(0.28, 0.7, text_Sawyer_end, transform=ax.transAxes, fontsize=14,
			bbox=props)
	text_UR_end = '\n'.join((
	r'$x=%.2f$' % (pose_UR_C[0], ),
	r'$y=%.2f$' % (pose_UR_C[1], )))
	ax.text(0.515, 0.7, text_UR_end, transform=ax.transAxes, fontsize=14,
			bbox=props)

def WebcamImageToMat(image):
    frame2=image.data.reshape([image.height, image.width, 3], order='C')
    return frame2

current_frame=np.zeros((100,100,3))

def new_frame(pipe_ep):
    global current_frame
    #Loop to get the newest frame
    while (pipe_ep.Available > 0):
        #Receive the packet
        image=pipe_ep.ReceivePacket()
        #Convert the packet to an image and set the global variable
        current_frame=WebcamImageToMat(image)
# Set up plot to call animate() function periodically

def animate2(i):
    im1.set_data(current_frame)

c=c_host.get_Webcams(0)

#Connect the pipe FrameStream to get the PipeEndpoint p
p=c.FrameStream.Connect(-1)

#Set the callback for when a new pipe packet is received to the
#new_frame function
p.PacketReceivedEvent+=new_frame
try:
    c.StartStreaming()
except: pass
im1 = ax1.imshow(current_frame)

ani = FuncAnimation(fig, animate, interval=100)
ani1 = FuncAnimation(plt.gcf(), animate2, interval=200)


plt.show()

p.Close()
c.StopStreaming()









