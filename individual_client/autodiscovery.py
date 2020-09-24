from RobotRaconteur.Client import *
import time

time.sleep(2)
res=RRN.FindServiceByType("com.robotraconteur.robotics.robot.Robot",
["rr+local","rr+tcp","rrs+tcp"])

print(res[0].Name)
print(res[0].ConnectionURL)