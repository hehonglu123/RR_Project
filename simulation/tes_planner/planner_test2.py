from RobotRaconteur.Client import *
import numpy as np
import argparse
import time

parser = argparse.ArgumentParser(description="Basic sanity test for planner")
parser.add_argument("--url",type=str,help="URL of planner service",default='rr+tcp://localhost:63158?service=tesseract')
parser.add_argument("--plot",default=False,action="store_true",help="Plot results")
args, _ = parser.parse_known_args()

robot = RRN.ConnectService('rr+local:///?nodeid=bfef6fcd-bac0-46b6-87f1-9b4e3560f9ac&service=robot')
c = RRN.ConnectService(args.url)


planning_constants = RRN.GetConstants("com.robotraconteur.robotics.planning",c)
planner_motion_type_code = planning_constants["PlannerMotionTypeCode"]

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)

JointWaypoint = RRN.GetStructureType('com.robotraconteur.robotics.planning.JointWaypoint',c)
CartesianWaypoint = RRN.GetStructureType('com.robotraconteur.robotics.planning.CartesianWaypoint',c)
Identifier = RRN.GetStructureType('com.robotraconteur.identifier.Identifier',c)
PoseDType = RRN.GetNamedArrayDType('com.robotraconteur.geometry.Pose',c)
SpatialVelocityDType = RRN.GetNamedArrayDType('com.robotraconteur.geometry.SpatialVelocity',c)

def create_planning_request(q_start,q_end):



    start_waypoint = JointWaypoint()
    end_waypoint = JointWaypoint()

    start_waypoint.joint_positions = q_start
    start_waypoint.motion_type = planner_motion_type_code["start"]

    end_waypoint.joint_positions = q_end
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
    planning_request.collision_safety_margin=0.001
    pose_dt = RRN.GetNamedArrayDType('com.robotraconteur.geometry.Pose',c)
    planning_request.tcp = np.zeros((1,),pose_dt)

    start_waypoint = RR.RobotRaconteurVarValue(start_waypoint, "com.robotraconteur.robotics.planning.JointWaypoint")
    goal_waypoint = RR.RobotRaconteurVarValue(end_waypoint, "com.robotraconteur.robotics.planning.JointWaypoint")

    planning_request.waypoints = [start_waypoint, goal_waypoint]

    return planning_request

def plan_to_pose(q_start,q_end):

    planning_request = create_planning_request(q_start,q_end)

    plan_generator = c.plan(planning_request)
    res = plan_generator.Next()
    plan_generator.Close()

    return res.joint_trajectory

def get_pose(p):
        
    H = np.zeros((1,),PoseDType)
    H[0]["position"]["x"] = p[0]
    H[0]["position"]["y"] = p[1]
    H[0]["position"]["z"] = p[2]
    H[0]["orientation"]["w"] = 0
    H[0]["orientation"]["x"] = 1
    H[0]["orientation"]["y"] = 0
    H[0]["orientation"]["z"] = 0
    return H

def execute_trajectory(traj):
    traj_gen = robot.execute_trajectory(traj)

    while (True):
        t = time.time()

        try:
            res = traj_gen.Next()
            print(res)
        except RR.StopIterationException:
            break

        print(hex(robot.robot_state.PeekInValue()[0].robot_state_flags))


halt_mode = robot_const["RobotCommandMode"]["halt"]
trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]

robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = trajectory_mode

q1=np.zeros((7,))
q1[0]=np.pi/2

move_trajectory = plan_to_pose(robot.robot_state.PeekInValue()[0].joint_position,q1)
execute_trajectory(move_trajectory)

q2=np.zeros((7,))
q2[0]=-np.pi/2
move_trajectory = plan_to_pose(q1,q2)
execute_trajectory(move_trajectory)

