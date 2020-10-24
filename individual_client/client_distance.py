from RobotRaconteur.Client import *
import numpy as np



distance_sub=RRN.SubscribeService('rr+tcp://localhost:25522?service=Environment')

distance_report_wire=distance_sub.SubscribeWire("distance_report_wire")

while True:
	wire_value=distance_report_wire.TryGetInValue()
	print(wire_value[0])
	if wire_value[0]:
		distance_report=wire_value[1]['staubli']
		Closest_Pt=distance_report.Closest_Pt
		Closest_Pt_env=distance_report.Closest_Pt_env
		dist=distance_report.min_distance
		J2C=distance_report.J2C

		print(dist,J2C)