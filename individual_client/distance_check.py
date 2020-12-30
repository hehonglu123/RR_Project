import re, sys, traceback, os, yaml, time
import numpy as np
import numpy.testing as nptest
from RobotRaconteur.Client import *

sys.path.append('../')
from gazebo_model_resource_locator import GazeboModelResourceLocator
from tesseract.tesseract_scene_graph import SimpleResourceLocator, SimpleResourceLocatorFn

from tesseract.tesseract_environment import Environment
from tesseract.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond
from tesseract.tesseract_collision import ContactResultMap, ContactRequest, ContactTestType_ALL, ContactResultVector
from tesseract.tesseract_collision import flattenResults as collisionFlattenResults

#Connect to robot service
with open('../client_yaml/client_ur.yaml') as file:
    url_ur= yaml.load(file)['url']
with open('../client_yaml/client_sawyer.yaml') as file:
    url_sawyer= yaml.load(file)['url']
with open('../client_yaml/client_abb.yaml') as file:
    url_abb= yaml.load(file)['url']

ur_sub=RRN.SubscribeService(url_ur)
sawyer_sub=RRN.SubscribeService(url_sawyer)
abb_sub=RRN.SubscribeService(url_abb)
UR_state=ur_sub.SubscribeWire("robot_state")
Sawyer_state=sawyer_sub.SubscribeWire("robot_state")
ABB_state=abb_sub.SubscribeWire("robot_state")

#link and joint names in urdf
Sawyer_joint_names=["right_j0","right_j1","right_j2","right_j3","right_j4","right_j5","right_j6"]
UR_joint_names=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
Sawyer_link_names=["right_l0","right_l1","right_l2","right_l3","right_l4","right_l5","right_l6","right_l1_2","right_l2_2","right_l4_2","right_hand"]
UR_link_names=['UR_base_link',"shoulder_link","upper_arm_link","forearm_link","wrist_1_link","wrist_2_link","wrist_3_link","UR_link_7"]
ABB_joint_names=['ABB1200_joint_1','ABB1200_joint_2','ABB1200_joint_3','ABB1200_joint_4','ABB1200_joint_5','ABB1200_joint_6']
ABB_link_names=['ABB1200_base_link','ABB1200_link_1','ABB1200_link_2','ABB1200_link_3','ABB1200_link_4','ABB1200_link_5','ABB1200_link_6','ABB1200_link_7']

robot_state_list=[UR_state,Sawyer_state,ABB_state]
robot_link_list=[UR_link_names,Sawyer_link_names,ABB_link_names]
robot_joint_list=[UR_joint_names,Sawyer_joint_names,ABB_joint_names]
num_robot=len(robot_state_list)
robot_dict={'ur':0,'sawyer':1,'abb':2}

locator = GazeboModelResourceLocator()

env = Environment()
urdf_path = FilesystemPath("../urdf/combined.urdf")
srdf_path = FilesystemPath("../urdf/combined.srdf")
assert env.init(urdf_path, srdf_path, locator)

checker = env.getDiscreteContactManager()

contact_distance=0.2
monitored_link_names = env.getLinkNames() 
checker.setActiveCollisionObjects(monitored_link_names)
checker.setContactDistanceThreshold(contact_distance)

#load calibration parameters
with open('../calibration/sawyer.yaml') as file:
    H_Sawyer    = np.array(yaml.load(file)['H'],dtype=np.float64)
with open('../calibration/ur.yaml') as file:
    H_UR        = np.array(yaml.load(file)['H'],dtype=np.float64)
with open('../calibration/abb.yaml') as file:
    H_ABB   = np.array(yaml.load(file)['H'],dtype=np.float64)   
env.changeJointOrigin("ur_pose", Isometry3d(H_UR))
env.changeJointOrigin("sawyer_pose", Isometry3d(H_Sawyer))
env.changeJointOrigin("abb_pose", Isometry3d(H_ABB))



def check():

    try:
        #update robot joints
        for i in range(num_robot):
            wire_packet=robot_state_list[i].TryGetInValue()
            #only update the ones online
            if wire_packet[0]:
                robot_joints=wire_packet[1].joint_position
                ## real ur only
                # if i==0:
                #     robot_joints[0]+=np.pi      #UR configuration
                env.setState(robot_joint_list[i], robot_joints)


        env_state = env.getCurrentState()
        checker.setCollisionObjectsTransform(env_state.link_transforms)
        result = ContactResultMap()

        checker.contactTest(result, ContactRequest(ContactTestType_ALL))
        result_vector = ContactResultVector()
        collisionFlattenResults(result,result_vector)

        distances = [r.distance for r in result_vector]
        nearest_points=[r.nearest_points for r in result_vector]

        names = [r.link_names for r in result_vector]

        
        for robot_name,robot_idx in robot_dict.items():
            min_distance=9
            min_index=-1
            Closest_Pt=[0.,0.,0.]
            Closest_Penv=[0.,0.,0.]
            J2C=0

            for i in range(len(distances)):

                #only 1 in 2 collision "objects"
                if (names[i][0] in robot_link_list[robot_idx] or names[i][1] in robot_link_list[robot_idx]) and distances[i]<min_distance and not (names[i][0] in robot_link_list[robot_idx] and names[i][1] in robot_link_list[robot_idx]):
                    min_distance=distances[i]
                    min_index=i

            if (min_index!=-1):
                if names[min_index][0] in robot_link_list[robot_idx] and names[min_index][1] in robot_link_list[robot_idx]:
                    stop=1

                elif names[min_index][0] in robot_link_list[robot_idx]:
                    J2C=robot_link_list[robot_idx].index(names[min_index][0])-1
                    Closest_Pt=nearest_points[min_index][0]
                    Closest_Penv=nearest_points[min_index][1]

                elif names[min_index][1] in robot_link_list[robot_idx]:
                    J2C=robot_link_list[robot_idx].index(names[min_index][1])-1
                    Closest_Pt=nearest_points[min_index][1]
                    Closest_Penv=nearest_points[min_index][0]


                if robot_idx==1:
                    J2C=Sawyer_link(J2C)-1

                
                print(robot_name,names[min_index][0],names[min_index][1])   
                            
    except:
        traceback.print_exc()

while True:
    check()
    time.sleep(0.1)