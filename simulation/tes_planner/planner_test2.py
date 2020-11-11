from RobotRaconteur.Client import *
import numpy as np
import argparse
import time

parser = argparse.ArgumentParser(description="Basic sanity test for planner")
parser.add_argument("--url",type=str,help="URL of planner service",default='rr+tcp://localhost:63158?service=tesseract')
parser.add_argument("--plot",default=False,action="store_true",help="Plot results")
args, _ = parser.parse_known_args()

c = RRN.ConnectService(args.url)


planning_constants = RRN.GetConstants("com.robotraconteur.robotics.planning",c)
planner_motion_type_code = planning_constants["PlannerMotionTypeCode"]


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

def plan(q_start,q_end):

    planning_request = create_planning_request(q_start,q_end)

    plan_generator = c.plan(planning_request)
    res = plan_generator.Next()
    plan_generator.Close()

    return res.joint_trajectory



q1=np.zeros((7,))
q1[0]=np.pi/2
q2=np.zeros((7,))
q2[0]=-np.pi/2


move_trajectory = plan(q1,q2)
print('trajectory 1 planned successfully')
move_trajectory = plan(q2,q1)
print('trajectory 2 planned successfully')

