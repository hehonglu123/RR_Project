import tesseract
import os
import re
import traceback
from tesseract_viewer import TesseractViewer
import numpy as np
import time
import sys
import yaml
sys.path.append('../../')
from gazebo_model_resource_locator import GazeboModelResourceLocator

with open("combined.urdf",'r') as f:
    combined_urdf = f.read()
with open("combined.srdf",'r') as f:
    combined_srdf = f.read()

pose=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],dtype=np.float64)


t = tesseract.Tesseract()

t.init(combined_urdf, combined_srdf, GazeboModelResourceLocator())

t_env = t.getEnvironment()

viewer = TesseractViewer()

viewer.update_environment(t_env, [0,0,0])


viewer.start_serve_background()


with open('../calibration/UR1.yaml') as file:
	H_UR1 		= np.array(yaml.load(file)['H'],dtype=np.float64)
with open('../calibration/UR2.yaml') as file:
	H_UR2 		= np.array(yaml.load(file)['H'],dtype=np.float64)
with open('../calibration/UR3.yaml') as file:
	H_UR3 		= np.array(yaml.load(file)['H'],dtype=np.float64)
with open('../calibration/UR4.yaml') as file:
	H_UR4 		= np.array(yaml.load(file)['H'],dtype=np.float64)

with open('../calibration/ABB1.yaml') as file:
	H_ABB1 	= np.array(yaml.load(file)['H'],dtype=np.float64)
with open('../calibration/ABB2.yaml') as file:
	H_ABB2 	= np.array(yaml.load(file)['H'],dtype=np.float64)
with open('../calibration/ABB3.yaml') as file:
	H_ABB3 	= np.array(yaml.load(file)['H'],dtype=np.float64)
with open('../calibration/ABB4.yaml') as file:
	H_ABB4 	= np.array(yaml.load(file)['H'],dtype=np.float64)

t_env.changeJointOrigin("UR1_pose", H_UR1)
t_env.changeJointOrigin("UR2_pose", H_UR2)
t_env.changeJointOrigin("UR3_pose", H_UR3)
t_env.changeJointOrigin("UR4_pose", H_UR4)
t_env.changeJointOrigin("ABB1_pose", H_ABB1)
t_env.changeJointOrigin("ABB2_pose", H_ABB2)
t_env.changeJointOrigin("ABB3_pose", H_ABB3)
t_env.changeJointOrigin("ABB4_pose", H_ABB4)

time.sleep(1)
viewer.update_environment(t_env, [0,0,0])

if sys.version_info[0] < 3:
    raw_input("press enter")
else:
    input("press enter")