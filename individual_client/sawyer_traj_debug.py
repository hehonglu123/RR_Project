from RobotRaconteur.Client import *
import time
import numpy as np

c = RRN.ConnectService('rr+tcp://[fe80::a2c:1efa:1c07:f043]:58654/?nodeid=8edf99b5-96b5-4b84-9acf-952af15f0918&service=robot')
#c = RRN.ConnectService('rr+local:///?nodename=sawyer_robot&service=sawyer')

robot_info = c.robot_info
print(robot_info)

#c.reset_errors()


#time.sleep(10)

print(c.robot_state.PeekInValue()[0].command_mode)

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)

joint_names = [j.joint_identifier.name for j in robot_info.joint_info]

halt_mode = robot_const["RobotCommandMode"]["halt"]
trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]
jog_mode = robot_const["RobotCommandMode"]["jog"]

JointTrajectoryWaypoint = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint",c)
JointTrajectory = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectory",c)

#c.reset_errors()

c.command_mode = halt_mode
time.sleep(0.1)
c.command_mode = jog_mode


state_w = c.robot_state.Connect()

state_w.WaitInValueValid()
state1 = state_w.InValue

waypoints = []

j_start = np.array([-0.42257715, -1.33285645, -0.21780469,  2.07934082,  0.03617383,  0.72125586,2.37041309])

j_end = np.array([-0.89141602, -0.93746875, -0.03746191,  1.50999902, -0.01694043,  0.97531738,  2.24939844])

jog_end = np.array([-0.92918359, -0.60203906, -0.12097461,  1.40458789,  0.15248828,  0.76406836,  2.03938867])

c.jog_freespace(j_start, 1.3*np.ones(7), True)
c.command_mode = halt_mode
time.sleep(0.1)
c.command_mode = trajectory_mode


for i in range(11):
    wp = JointTrajectoryWaypoint()
    wp.joint_position = (j_end - j_start)*(float(i)/10.0) + j_start
    wp.time_from_start = i/2.0
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
c.jog_freespace(jog_end, 1.3*np.ones(7), True)



