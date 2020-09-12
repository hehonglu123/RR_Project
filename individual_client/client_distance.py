from RobotRaconteur.Client import *
import numpy as np
distance_inst=RRN.ConnectService('rr+tcp://localhost:25522?service=Environment')

distance_report = distance_inst.distance_check('ur')
Closest_Pt=distance_report.Closest_Pt
Closest_Pt_env=distance_report.Closest_Pt_env
dist=distance_report.min_distance
J2C=distance_report.J2C

print(dist,J2C)