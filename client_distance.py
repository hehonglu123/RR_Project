from RobotRaconteur.Client import *
distance_inst=RRN.ConnectService('rr+tcp://127.0.0.1:25522?service=Environment')


distance_report=distance_inst.distance_check("UR")
print(distance_report.Closest_Pt)
print(distance_report.Closest_Pt_env)
print(distance_report.min_distance)
print(distance_report.robot_link_name)

transformations=distance_inst.transformations
print(transformations[0].name,transformations[0].H.reshape((transformations[0].row,transformations[0].column)))
print(transformations[1].name,transformations[1].H.reshape((transformations[1].row,transformations[1].column)))


