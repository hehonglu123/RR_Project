import numpy as np

import sys, time, yaml, argparse
from scipy.optimize import leastsq

kinect_cod=np.array([[      510.3,      172.95],
 [     336.18,      151.07],
 [     518.44,      461.38],
 [     353.81,      448.76],
 [     567.04,      212.72],
 [     424.01,      206.58],
 [     582.64,      430.93],
 [     433.62,      430.02],
 [     477.64,      157.43],
 [     326.57,      162.75],
 [     494.24,      475.36],
 [     350.86,      484.64]])

cognex_cod=np.array([[-.133,.272],
[-.411,.311],
[-.116,-.202],
[-.393,-.176],
[-.032,.208],
[-.269,.220],
[-.006,-.153],
[-.256,-.147],
[-.1755,.292],
[-.428,.290],
[-.153,-.222],
[-.398,-.235]])

with open('intrinsic.yaml') as file:
    intrinsic = np.array(yaml.load(file)['W'],dtype=np.float64)



def my_func(x,cog,kinect):

    R=np.array([[np.cos(x[0]),-np.sin(x[0])],[np.sin(x[0]),np.cos(x[0])]])
    result=np.dot(R,kinect)-cog+np.array([[x[1]],[x[2]]])
    return result.flatten()


def calibrate(cog,kinect): 
    result,r = leastsq(func=my_func,x0=[0,0,0],args=(np.transpose(np.array(cog)),np.transpose(np.array(kinect))))
    H=np.zeros((3,3))
    H[0][0]=np.cos(result[0])
    H[0][1]=-np.sin(result[0])
    H[1][0]=np.sin(result[0])
    H[1][1]=np.cos(result[0])
    H[0][-1]=-result[1]
    H[1][-1]=-result[2]
    H[-1][-1]=1
    return H


kinect_cod-=intrinsic[:2,-1]
kinect_cod/=600.
kinect_cod[:,1]*=-1

H=calibrate(kinect_cod, cognex_cod)
print(H)
dict_file={'H':H.tolist()}
with open('kinect.yaml', 'w') as file:
    yaml.dump(dict_file, file)

for i in range(len(cognex_cod)):
    cood=np.array([[kinect_cod[i][0]],[kinect_cod[i][1]],[1]])
    act=np.array([[cognex_cod[i][0]],[cognex_cod[i][1]],[1]])
    diff=np.dot(H,cood)-act
    print(diff)