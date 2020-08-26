
from js import print_div
from js import print_flags
from js import print_joints
from js import document
from js import ImageData
from RobotRaconteur.Client import *
import numpy as np
import traceback

from js import Plotly


def H_inv(H):							#inverse the homogeneous transformation matrix
	R=H[:2,:2]
	R=np.transpose(R)
	d=np.transpose(np.array([H[:2,2]]))
	d=-np.dot(R,d)
	H=np.concatenate((np.concatenate((R,d),axis=1),np.array([[0,0,1]])),axis=0)
	return H


def new_frame(pipe_ep):
	global canvas, ctx
	#Loop to get the newest frame
	while (pipe_ep.Available > 0):
		#Receive the packet
		image=pipe_ep.ReceivePacket()
		#Convert the packet to an image and set the global variable
		
		if (canvas == None):
			canvas = document.getElementById("image")
			ctx = canvas.getContext("2d")
		
		imageBytes=np.zeros(4*image.width*image.height, dtype=np.uint8)		#dtype essential here, IndexSizeError
		imageBytes[3::4] = 255
		imageBytes[0::4] = image.data[2::3]
		imageBytes[1::4] = image.data[1::3]
		imageBytes[2::4] = image.data[0::3]


		image_data=ImageData.new(bytes(imageBytes),image.width,image.height)
		ctx.putImageData(image_data, 0, 0,0,0,320,240)

async def client_plotly():

	uname=document.getElementById("uname").value
	psw=document.getElementById("psw").value

	credentials={"password":RR.RobotRaconteurVarValue(psw,"string")}

	try:
		inst=await RRN.AsyncConnectService('rr+ws://128.113.224.144:52222/?service=SmartCam',uname,credentials,None,None)
		Sawyer=await RRN.AsyncConnectService('rr+ws://128.113.224.144:58653?service=sawyer',None,None,None,None)
		UR=await RRN.AsyncConnectService('rr+tcp://128.113.224.144:58652?service=ur_robot',None,None,None,None)
		c_host=await RRN.AsyncConnectService('rr+ws://128.113.224.144:2366?service=Webcam',uname,credentials,None,None)
		c= await c_host.async_get_Webcams(0,None)

		p= await c.FrameStream.AsyncConnect(-1,None)
		global canvas, ctx
		canvas = document.getElementById("image")
		ctx = canvas.getContext("2d")
		print_div("Running!")
		p.PacketReceivedEvent+=new_frame
		c.async_StartStreaming(None)

		#robot state flag
		robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", Sawyer)
		state_flags_enum = robot_const['RobotStateFlags']

		while True:

			await plot(UR,Sawyer, inst, state_flags_enum)
			# await RRN.AsyncSleep(0.01,None)

	except:
		
		print_div(traceback.format_exc())
		raise

H_Sawyer=np.array([[-0.007,0.9704,0.5056],[-0.9548,-0.016,0.1196],[0,0,1]])
H_UR=np.array([[9.62950075e-01, 6.21989212e-02,  -0.184218921],[-5.85342469e-02,  9.57014647e-01, -0.350103328],[0,0,1]])

H_S_C=H_inv(H_Sawyer)
H_UR_C=H_inv(H_UR)


async def plot(UR, Sawyer, inst, state_flags_enum):

	xs = []
	ys = []

	# Add x and y to lists
	for obj in await inst.async_get_objects(None):
		xs.append(obj.x/1000.)
		ys.append(obj.y/1000.)

	sawyer_state = await Sawyer.robot_state.AsyncPeekInValue(None)
	ur_state = await UR.robot_state.AsyncPeekInValue(None)

	pose_Sawyer=sawyer_state[0].kin_chain_tcp
	q_Sawyer=sawyer_state[0].joint_position

	q_UR=ur_state[0].joint_position
	pose_UR=ur_state[0].kin_chain_tcp


	# Draw x and y lists
	pose_Sawyer_C=np.dot(H_S_C,np.array([[pose_Sawyer[0]['position']['x']],[pose_Sawyer[0]['position']['y']],[1]]))
	pose_UR_C=np.dot(H_UR_C,np.array([[pose_UR[0]['position']['x']],[pose_UR[0]['position']['y']],[1]]))


	objects={ 'y': ys, 'x': xs ,'mode':'markers','name':'objects','type':'scatter','marker':{'size':10,'color':'#000000'}}
	UR5_robot={ 'y': pose_UR_C[1], 'x': pose_UR_C[0] ,'mode':'markers','name':'UR5_robot','type':'scatter','marker':{'size':10,'color':'#27e3d3'}}
	Sawyer_robot={ 'y': pose_Sawyer_C[1], 'x': pose_Sawyer_C[0] ,'mode':'markers','name':'Sawyer_robot','type':'scatter','marker':{'size':10,'color':'#e31010'}}

	Plotly.react('plot',[objects,UR5_robot, Sawyer_robot])



	#robot flags
	# flags_text_sawyer = "Sawyer State Flags:<br>"
	# for flag_name, flag_code in state_flags_enum.items():
	# 	if flag_code & sawyer_state[0].robot_state_flags != 0:
	# 		flags_text_sawyer += flag_name + "<br>"
	# flags_text_UR = "UR State Flags:<br>"
	# for flag_name, flag_code in state_flags_enum.items():
	# 	if flag_code & ur_state[0].robot_state_flags != 0:
	# 		flags_text_UR += flag_name + "<br>"
	# print_flags(flags_text_sawyer,flags_text_UR)

	# joints_text_sawyer="Sawyer Joint Positions:<br>"
	# for i in q_Sawyer:
	# 	joints_text_sawyer+= "%.2f<br>" % np.rad2deg(i)
	# joints_text_UR="UR Joint Positions:<br>"
	# for i in q_UR:
	# 	joints_text_UR+= "%.2f<br>" % np.rad2deg(i)
	# print_joints(joints_text_sawyer, joints_text_UR)



	



RR.WebLoop.run(client_plotly())
