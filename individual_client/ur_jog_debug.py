from RobotRaconteur.Client import *
import sys, time, yaml, traceback
import numpy as np
sys.path.append('../')
from vel_emulate_sub import EmulatedVelocityControl


robot_name='ur'
url='rr+tcp://[fe80::76d6:e60f:27f6:1e3e]:58653/?nodeid=55ade648-a8c2-4775-a7ec-645acea83525&service=robot'
robot_sub=RRN.SubscribeService(url)
c=robot_sub.GetDefaultClientWait(1)

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)

joint_names = [j.joint_identifier.name for j in c.robot_info.joint_info]

halt_mode = robot_const["RobotCommandMode"]["halt"]
trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]
jog_mode = robot_const["RobotCommandMode"]["jog"]

JointTrajectoryWaypoint = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint",c)
JointTrajectory = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectory",c)



c.command_mode = halt_mode
time.sleep(0.1)
c.command_mode = jog_mode


##robot wire
cmd_w = robot_sub.SubscribeWire("position_command")
state_w = robot_sub.SubscribeWire("robot_state")

waypoints = []

j_start=np.array([-0.55219562, -1.60498046,  2.1155784,  -2.08339566, -1.57870295, -1.85687867])
j_end=np.array([ 0.69338194, -1.70667589,  2.11468408, -2.0041226,  -1.56872805, -1.79422005])
jog_end=np.array([ 0.74692304, -1.40931117,  2.29963487, -2.4610044,  -1.57080458,-2.42770829])



c.jog_freespace(j_start, np.ones(6), True)
c.command_mode = halt_mode
time.sleep(0.1)
c.command_mode = trajectory_mode


for i in range(11):
    wp = JointTrajectoryWaypoint()
    wp.joint_position = (j_end - j_start)*(float(i)/10.0) + j_start
    wp.time_from_start = i/10.0
    waypoints.append(wp)

traj = JointTrajectory()
traj.joint_names = joint_names
traj.waypoints = waypoints

c.speed_ratio = 1

traj_gen = c.execute_trajectory(traj)

while (True):
    t = time.time()

    robot_state = state_w.InValue
    try:
        res = traj_gen.Next()
        print(res)
    except RR.StopIterationException:
        break

    print(hex(c.robot_state.PeekInValue()[0].robot_state_flags))

c.command_mode = halt_mode
time.sleep(0.1)
c.command_mode = jog_mode
c.jog_freespace(jog_end, np.ones(6), True)
