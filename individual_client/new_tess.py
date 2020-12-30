import re, sys, traceback, os, yaml
import numpy as np
import numpy.testing as nptest

sys.path.append('../')
from gazebo_model_resource_locator import GazeboModelResourceLocator
from tesseract.tesseract_scene_graph import SimpleResourceLocator, SimpleResourceLocatorFn

from tesseract.tesseract_environment import Environment
from tesseract.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond
from tesseract.tesseract_collision import ContactResultMap, ContactRequest, ContactTestType_ALL, ContactResultVector
from tesseract.tesseract_collision import flattenResults as collisionFlattenResults

#link and joint names in urdf
Sawyer_joint_names=["right_j0","right_j1","right_j2","right_j3","right_j4","right_j5","right_j6"]
UR_joint_names=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
Sawyer_link_names=["right_l0","right_l1","right_l2","right_l3","right_l4","right_l5","right_l6","right_l1_2","right_l2_2","right_l4_2","right_hand"]
UR_link_names=['UR_base_link',"shoulder_link","upper_arm_link","forearm_link","wrist_1_link","wrist_2_link","wrist_3_link"]
ABB_joint_names=['ABB1200_joint_1','ABB1200_joint_2','ABB1200_joint_3','ABB1200_joint_4','ABB1200_joint_5','ABB1200_joint_6']
ABB_link_names=['ABB1200_base_link','ABB1200_link_1','ABB1200_link_2','ABB1200_link_3','ABB1200_link_4','ABB1200_link_5','ABB1200_link_6']

def main():
    
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


    joint_angles = np.ones((6,),dtype=np.float64) * 0.1
    joint_angles[1] = 2
    joint_angles[2] = 2

    env.setState(ABB_joint_names, joint_angles)

    env_state = env.getCurrentState()
    checker.setCollisionObjectsTransform(env_state.link_transforms)

    result = ContactResultMap()
    checker.contactTest(result, ContactRequest(ContactTestType_ALL))
    result_vector = ContactResultVector()
    collisionFlattenResults(result,result_vector)
    
    for r in result_vector:
        print(r.link_names[0] + "," + r.link_names[1] + " " + str(r.distance))
    
if __name__ == "__main__":
    main()