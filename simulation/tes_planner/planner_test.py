from RobotRaconteur.Client import *
import numpy as np
import argparse, time

parser = argparse.ArgumentParser(description="Basic sanity test for planner")
parser.add_argument("--url",type=str,help="URL of planner service",default='rr+tcp://localhost:63158?service=tesseract')
parser.add_argument("--plot",default=False,action="store_true",help="Plot results")
parser.add_argument("--robot-name",type=str,help="List of camera names separated with commas")

args, _ = parser.parse_known_args()
robot_name=args.robot_name

c = RRN.ConnectService(args.url)

planning_constants = RRN.GetConstants("com.robotraconteur.robotics.planning",c)
planner_motion_type_code = planning_constants["PlannerMotionTypeCode"]

JointWaypoint = RRN.GetStructureType('com.robotraconteur.robotics.planning.JointWaypoint',c)
Identifier = RRN.GetStructureType('com.robotraconteur.identifier.Identifier',c)

start_waypoint = JointWaypoint()
end_waypoint = JointWaypoint()

start_waypoint.joint_positions = np.zeros((7,))
start_waypoint.joint_positions[0]=np.pi/2
start_waypoint.joint_positions[1]=-.7

# start_waypoint.joint_positions=np.array([ 0.31708022, -1.35813582,  1.00756832,  1.84204846,  2.97610036, -1.2160278,
#   1.37919743]).reshape((7,))
start_waypoint.motion_type = planner_motion_type_code["start"]

end_waypoint.joint_positions = np.zeros((7,))
end_waypoint.joint_positions[0]=-np.pi/2
end_waypoint.joint_positions[1]=-1.
# end_waypoint.joint_positions=np.array([ 0.31708022, -1.35813582,  1.00756832,  1.84204846,  2.97610036, -1.2160278,
#   1.37919743]).reshape((7,))


end_waypoint.motion_type = planner_motion_type_code["freespace"]
end_waypoint.time_from_start = 5


planning_request = RRN.NewStructure('com.robotraconteur.robotics.planning.PlanningRequest',c)
planning_request.device = Identifier()
planning_request.device.name = "right_arm"
uuid_dt = RRN.GetNamedArrayDType('com.robotraconteur.uuid.UUID',c)
planning_request.device.uuid=np.zeros((1,), uuid_dt)
planning_request.planner_algorithm = Identifier()
planning_request.planner_algorithm.name = "trajopt"
planning_request.planner_algorithm.uuid =np.zeros((1,), uuid_dt)
planning_request.filter_algorithm = Identifier()
planning_request.filter_algorithm.name = "iterative_spline"
planning_request.filter_algorithm.uuid =np.zeros((1,), uuid_dt)

box_dt = RRN.GetNamedArrayDType('com.robotraconteur.geometry.Box',c)
bounds = np.zeros((1,),box_dt)
bounds[0]["origin"]["x"] = -10
bounds[0]["origin"]["y"] = -10
bounds[0]["origin"]["z"] = -10
bounds[0]["size"]["width"] = 20
bounds[0]["size"]["height"] = 20
bounds[0]["size"]["depth"] = 20
planning_request.workspace_bounds = bounds
planning_request.collision_check=True
planning_request.collision_safety_margin=0.25
pose_dt = RRN.GetNamedArrayDType('com.robotraconteur.geometry.Pose',c)
planning_request.tcp = np.zeros((1,),pose_dt)

start_waypoint = RR.RobotRaconteurVarValue(start_waypoint, "com.robotraconteur.robotics.planning.JointWaypoint")
goal_waypoint = RR.RobotRaconteurVarValue(end_waypoint, "com.robotraconteur.robotics.planning.JointWaypoint")

planning_request.waypoints = [start_waypoint, goal_waypoint]

plan_generator = c.plan(planning_request)
res = plan_generator.Next()
plan_generator.Close()

joint_trajectory=res.joint_trajectory

#auto discovery
res=RRN.FindServiceByType("com.robotraconteur.robotics.robot.Robot",
["rr+local","rr+tcp","rrs+tcp"])
url=None
for serviceinfo2 in res:
    if robot_name in serviceinfo2.NodeName:
        url=serviceinfo2.ConnectionURL
        break
if url==None:
    print('service not found')
    sys.exit()

robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]
jog_mode = robot_const["RobotCommandMode"]["jog"]

robot.command_mode = halt_mode
time.sleep(0.01)
robot.command_mode = jog_mode
robot.jog_freespace(joint_trajectory.waypoints[0].joint_position,np.ones(7),True)


robot.command_mode = halt_mode
time.sleep(0.01)
robot.command_mode = trajectory_mode


JointTrajectoryWaypoint = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint",robot)
JointTrajectory = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectory",robot)
waypoints = []

for i in range(len(joint_trajectory.waypoints)):
    wp = JointTrajectoryWaypoint()
    wp.joint_position = joint_trajectory.waypoints[i].joint_position
    wp.time_from_start = joint_trajectory.waypoints[i].time_from_start
    waypoints.append(wp)

traj = JointTrajectory()
traj.joint_names = [j.joint_identifier.name for j in robot.robot_info.joint_info]
traj.waypoints = waypoints



traj_gen = robot.execute_trajectory(traj)

while (True):
    t = time.time()

    try:
        res = traj_gen.Next()
        print(res)
    except RR.StopIterationException:
        break



if args.plot:
    import matplotlib
    import matplotlib.pyplot as plt

    n_waypoints = len(joint_trajectory.waypoints)

    time = np.zeros((n_waypoints,),dtype=np.float)
    joints = np.zeros((n_waypoints,len(joint_trajectory.joint_names)),dtype=np.float)
    for i in range(n_waypoints):
        time[i] = joint_trajectory.waypoints[i].time_from_start
        joints[i,:] = joint_trajectory.waypoints[i].joint_position
    print(joints)

    plt.plot(time,joints)
    plt.show()

