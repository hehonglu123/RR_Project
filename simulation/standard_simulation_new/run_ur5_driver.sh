#!/bin/sh
cd ~/GazeboModelRobotRaconteurDriver-2021-04-18
# RR URL: rr+tcp://localhost:58653?service=robot

dotnet GazeboModelRobotRaconteurDriver.dll --gazebo-url=rr+tcp://localhost:11346/?service=GazeboServer --robotraconteur-tcp-port=58653 --robotraconteur-nodename=ur5_robot --model-name=ur --robot-info-file=/mnt/c/Users/hehon/Desktop/RobotRaconteur/RR_Project/simulation/standard_simulation_new/config/ur5_robot_default_config.yml
