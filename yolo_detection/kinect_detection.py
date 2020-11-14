import argparse, os, copy, torch
from cv2 import aruco
import cv2, sys, time, traceback
import numpy as np


from models.experimental import attempt_load
from utils.general import non_max_suppression, plot_images, output_to_target

from RobotRaconteur.Client import *


class create_impl(object):
    def __init__(self):
        #initialize detection parameters
        self.BACKGROUND=150
        self.RESOLUTION=1
        raw_image_width=1280
        raw_image_height=720
        self.detection_start_r=round(.15*raw_image_height)
        self.detection_end_r=round(.75*raw_image_height)
        self.detection_start_c=round(.23*raw_image_width)
        self.detection_end_c=round(.48*raw_image_width)
        self.offset=np.zeros((4,3))
        self.offset[:,0]=self.detection_start_c
        self.offset[:,1]=self.detection_start_r
        self.device = torch.device('cuda')
        self.detection_width=192
        self.detection_height=256

        #transfer to cognex frame
        self.H=np.array([[ 0.99995151, -0.00984728, 0.08885063],
                        [ 0.00984728,  0.99995151, -0.03708229],
                        [ 0.,          0.,          1.        ]])
        self.f=600
        self.center_c=640
        self.center_r=360

        # device = torch.device('cpu')
        # Load model
        self.model = attempt_load("my_test/kinect_weight.pt",map_location=self.device)  # load FP32 model
        self.names = {k: v for k, v in enumerate(self.model.names if hasattr(self.model, 'names') else self.model.module.names)}

        # Configure
        self.model.eval()

        #image pipe setting
        self.current_frame=None
        url='rr+tcp://localhost:2355?service=Webcam'
        # if (len(sys.argv)>=2):
        #     url=sys.argv[1]

        #Startup, connect, and pull out the camera from the objref    
        c_host=RRN.ConnectService(url)

        self.c=c_host.get_Webcams(0)

        #Connect the pipe FrameStream to get the PipeEndpoint p
        self.p=self.c.FrameStream.Connect(-1)

        #Set the callback for when a new pipe packet is received to the
        #new_frame function
        self.p.PacketReceivedEvent+=self.new_frame
        try:
            self.c.StartStreaming()
        except: pass
        #timing
        self.now=time.time()

        #orientation checking param
        gt_soap=cv2.cvtColor(cv2.imread("orientation_check/soap.jpg"),cv2.COLOR_BGR2GRAY)
        gt_toothpaste=cv2.cvtColor(cv2.imread("orientation_check/toothpaste.jpg"),cv2.COLOR_BGR2GRAY)
        gt_perfume=cv2.cvtColor(cv2.imread("orientation_check/perfume.jpg"),cv2.COLOR_BGR2GRAY)
        self.max_size=38
        gt_soap_tensor=np.zeros((int(360/self.RESOLUTION),self.max_size,self.max_size))
        gt_toothpaste_tensor=np.zeros((int(360/self.RESOLUTION),self.max_size,self.max_size))
        gt_perfume_tensor=np.zeros((int(360/self.RESOLUTION),self.max_size,self.max_size))

        for i in range(0,360,self.RESOLUTION):
            gt_soap_tensor[int(i/self.RESOLUTION)]=self.square(self.rotate_image(gt_soap,i),self.max_size)
            gt_toothpaste_tensor[int(i/self.RESOLUTION)]=self.square(self.rotate_image(gt_toothpaste,i),self.max_size)
            gt_perfume_tensor[int(i/self.RESOLUTION)]=self.square(self.rotate_image(gt_perfume,i),self.max_size)

        self.gt_dict={0:gt_toothpaste_tensor,1:gt_perfume_tensor,2:gt_soap_tensor}
        
        #initialize objrecog structures
        self._object_recognition_sensor_data=None
        self.object_recognition_sensor_data_data=RRN.NewStructure("edu.robotraconteur.objectrecognition.ObjectRecognitionSensorData")
        self.object_recognition_sensor_data_data.recognized_objects=RRN.NewStructure("edu.robotraconteur.objectrecognition.RecognizedObjects")  #recognized_objects

        self.recognized_object=RRN.NewStructure("edu.robotraconteur.objectrecognition.RecognizedObject")
        self.recognized_object.recognized_object=RRN.NewStructure("com.robotraconteur.identifier.Identifier")                   #name,uuid
        uuid_dtype=RRN.GetNamedArrayDType("com.robotraconteur.uuid.UUID")
        self.recognized_object.recognized_object.uuid=np.zeros((1,),dtype=uuid_dtype)
        self.recognized_object.pose=RRN.NewStructure("com.robotraconteur.geometry.NamedPoseWithCovariance")
        self.recognized_object.pose.pose=RRN.NewStructure("com.robotraconteur.geometry.NamedPose")
        pose_dtype=RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose")
        self.recognized_object.pose.pose.pose=np.zeros((1,),dtype=pose_dtype)

        #initialize detection obj map
        self.detection_objects={}
        self.detection_obj=RRN.NewStructure("edu.robotraconteur.objectrecognition.detection_obj")
        self.models=['tp','pf','sp','bt']
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

    #Function to take the data structure returned from the Webcam service
    #and convert it to an OpenCV array
    def WebcamImageToMat(self,image):
        frame2=image.data.reshape([image.height, image.width, 3], order='C')
        return frame2


    #This function is called when a new pipe packet arrives
    def new_frame(self,pipe_ep):
        #Loop to get the newest frame
        while (pipe_ep.Available > 0):
            try:
                #Receive the packet
                image=pipe_ep.ReceivePacket()
                #Convert the packet to an image and set the global variable
                tmp_frame=self.WebcamImageToMat(image)
                tmp_frame=self.aruco_process(tmp_frame)
                detection_frame=self.test(tmp_frame[self.detection_start_r:self.detection_end_r,self.detection_start_c:self.detection_end_c,:])
                tmp_frame[self.detection_start_r:self.detection_end_r,self.detection_start_c:self.detection_end_c,:]=cv2.resize(detection_frame,(-self.detection_start_c+self.detection_end_c,-self.detection_start_r+self.detection_end_r))
                self.current_frame=tmp_frame
                # print(time.time()-self.now)
                self.now=time.time()
            except:
                traceback.print_exc()
    def aruco_process(self,frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()
        parameters.minMarkerPerimeterRate=0.00001
        parameters.adaptiveThreshConstant=20
        # parameters.minMarkerDistanceRate=0.005
        parameters.adaptiveThreshWinSizeMin=5
        parameters.adaptiveThreshWinSizeMax=10
        parameters.adaptiveThreshWinSizeStep=1

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        return frame_markers
    def square(self,im,desired_size):
        image=self.BACKGROUND*np.ones((desired_size, desired_size),np.uint8)
        shape_diff=[desired_size-im.shape[0],desired_size-im.shape[1]]
        start=[round(shape_diff[0]/2),round(shape_diff[1]/2)]
        end=[desired_size-round(shape_diff[0]/2),desired_size-round(shape_diff[1]/2)]
        image[start[0]:end[0],start[1]:end[1]]=cv2.resize(im,(end[1]-start[1],end[0]-start[0]))

        return image
    def rotate_image(self,mat, angle):
        """
        Rotates an image (angle in degrees) and expands image to avoid cropping
        """

        height, width = mat.shape[:2] # image shape has 3 dimensions

        image_center = (width/2, height/2) # getRotationMatrix2D needs coordinates in reverse order (width, height) compared to shape

        rotation_mat = cv2.getRotationMatrix2D(image_center, angle, 1.)

        # rotation calculates the cos and sin, taking absolutes of those.
        abs_cos = abs(rotation_mat[0,0]) 
        abs_sin = abs(rotation_mat[0,1])

        # find the new width and height bounds
        bound_w = int(height * abs_sin + width * abs_cos)
        bound_h = int(height * abs_cos + width * abs_sin)

        # subtract old image center (bringing image back to origo) and adding the new image center coordinates
        rotation_mat[0, 2] += bound_w/2 - image_center[0]
        rotation_mat[1, 2] += bound_h/2 - image_center[1]

        # rotate image with the new bounds and translated rotation matrix
        rotated_mat = cv2.warpAffine(mat, rotation_mat, (bound_w, bound_h),cv2.BORDER_CONSTANT,borderValue=self.BACKGROUND)
        return rotated_mat


    def orientation(self,gt_tensor,bound_img):
        return self.RESOLUTION*np.argmin(np.linalg.norm(gt_tensor-bound_img,axis=(1,2)))


    def test(self,img_raw):  # number of logged images

        
        #found objects
        found=np.zeros((4,3))

        try:
            img_resize=cv2.resize(img_raw,(self.detection_width,self.detection_height))
            img=cv2.cvtColor(img_resize, cv2.COLOR_BGR2RGB)
            img=img.transpose(2,0,1)
            img=torch.tensor(img.reshape((1,3,self.detection_height,self.detection_width))).float().to(self.device)
        except:
            traceback.print_exc()

        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        nb, _, height, width = img.shape  # batch size, channels, height, width

        # Run model
        inf_out, train_out = self.model(img)  # inference and training outputs

        # Run NMS
        output = non_max_suppression(inf_out, conf_thres=0.8)

        if output[0]==None:
            return img_raw
        out=output[0].cpu().numpy()
        #sort last column
        out[out[:,-1].argsort()]
        out_filter=[]

        
        for i in range(len(out)):
            obj=out[i]
            if abs(obj[3]-obj[1])<2 or abs(obj[2]-obj[0])<2 or obj[0]<0 or obj[1]<0 or obj[2]<0 or obj[3]<0:
                out_filter.append(i)
                continue
            #filter out obj not in region
            if obj[-1]==0 and (obj[0]<.45*width or obj[1]>0.3*height):
                out_filter.append(i)
                continue
            if obj[-1]==1 and (obj[0]>.45*width or obj[1]>0.3*height):
                out_filter.append(i)
                continue
            if obj[-1]==2 and (obj[0]<.5*width or obj[1]<0.6*height):
                out_filter.append(i)
                continue
            if obj[-1]==3 and (obj[0]>.5*width or obj[1]<0.6*height):
                out_filter.append(i)
                continue
            #no need to check orientation for bottle
            if found[int(obj[-1])][0]:
                continue
            #check orientation
            if obj[-1]!=3:
                img_crop=img_resize[int(obj[1]):int(obj[3]),int(obj[0]):int(obj[2]),:]
                angle=self.orientation(self.gt_dict[obj[-1]],self.square(cv2.cvtColor(img_crop,cv2.COLOR_BGR2GRAY),self.max_size))
            else:
                angle=0

            found[int(obj[-1])]=np.array([(obj[0]+obj[2])/2,(obj[1]+obj[3])/2,angle])

        out=np.delete(out,out_filter,axis=0)

        img_out=cv2.cvtColor(plot_images(img, output_to_target([torch.tensor(out)], width, height),fname=None, names=self.names), cv2.COLOR_RGB2BGR)
        for i in range(4):
            obj=out[i]
            if found[int(obj[-1])][0]:
                img_out= cv2.putText(img_out, str(int(found[i][-1])), (int(found[i][0]),int(found[i][1])),fontFace=cv2.FONT_HERSHEY_COMPLEX_SMALL,fontScale=.5,color=(0,0,0))
                #my type
                kinect_cood_c=((found[i][0]*(self.detection_end_c-self.detection_start_c)/self.detection_width+self.detection_start_c)-self.center_c)/self.f
                kinect_cood_r=-((found[i][1]*(self.detection_end_r-self.detection_start_r)/self.detection_height+self.detection_start_r)-self.center_r)/self.f
                trans=np.dot(self.H,np.array([[kinect_cood_c],[kinect_cood_r],[1]]))
                list(self.detection_objects.values())[i].x = trans[0][0]
                list(self.detection_objects.values())[i].y = trans[1][0]
                list(self.detection_objects.values())[i].angle=found[i][-1]-90
                list(self.detection_objects.values())[i].detected=True
            else:
                self.detection_objects.values()[i].detected=False

        found[:,0]*=(self.detection_end_c-self.detection_start_c)/self.detection_width
        found[:,1]*=(self.detection_end_r-self.detection_start_r)/self.detection_height
        found+=self.offset     


        #pass to RR wire
        self.detection_wire.OutValue=self.detection_objects  

        return img_out


def main():
    #Accept the names of the webcams and the nodename from command line
    parser = argparse.ArgumentParser(description="kinect object detection service")
    parser.add_argument("--view",type=bool,default=False)
    args, _ = parser.parse_known_args()
    with RR.ServerNodeSetup("detection_Service", 52222) as node_setup:

        cwd = os.getcwd()
        os.chdir('/home/rpi/catkin_ws/src/robotraconteur_companion/robdef/group1')
        RRN.RegisterServiceTypesFromFiles(['edu.robotraconteur.objectrecognition.robdef'],True) 
        os.chdir(cwd)
        detection_inst=create_impl()
        try:
            RRN.RegisterService("detection", "edu.robotraconteur.objectrecognition.ObjectRecognitionSensor", detection_inst)
        except:
            traceback.print_exc()
        
        if not args.view:
            input("press enter to quit")
        while args.view:
            #Just loop resetting the frame
            #This is not ideal but good enough for demonstration

            if (not detection_inst.current_frame is None):
                cv2.imshow("Detection Image",detection_inst.current_frame)
            if cv2.waitKey(50)!=-1:
                break
        cv2.destroyAllWindows()

        detection_inst.p.Close()
        detection_inst.c.StopStreaming()

if __name__ == '__main__':
    
    main()

