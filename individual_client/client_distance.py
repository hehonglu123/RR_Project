from RobotRaconteur.Client import *
import numpy as np
distance_inst=RRN.ConnectService('rr+tcp://127.0.0.1:25522?service=Environment')


print(np.array(distance_inst.distance_matrix).reshape((distance_inst.num_robot,distance_inst.num_robot)))