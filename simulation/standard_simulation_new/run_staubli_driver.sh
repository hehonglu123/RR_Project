#!/bin/sh
cd ~/GazeboModelRobotRaconteurDriver-2021-04-18
# RR URL: rr+tcp://localhost:58656?service=robot

dotnet GazeboModelRobotRaconteurDriver.dll --gazebo-url=rr+tcp://localhost:11346/?service=GazeboServer --robotraconteur-tcp-port=58656 --robotraconteur-nodename=staubli_robot --model-name=staubli --robot-info-file=/mnt/c/Users/hehon/Desktop/RobotRaconteur/RR_Project/simulation/standard_simulation_new/config/staubli_robot_default_config.yml
