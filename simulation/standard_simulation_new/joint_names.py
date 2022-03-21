
from RobotRaconteur.Client import *
#auto discovery
res=RRN.FindServiceByType("com.robotraconteur.robotics.robot.Robot",
["rr+local","rr+tcp","rrs+tcp"])
url=None
for serviceinfo2 in res:
    if 'staubli' in serviceinfo2.NodeName:
        url=serviceinfo2.ConnectionURL
        break
if url==None:
    print('service not found')
    sys.exit()
robot=RRN.ConnectService(url)
joint_names = [j.joint_identifier.name for j in robot.robot_info.joint_info]
print(joint_names)