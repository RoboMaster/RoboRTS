#!/bin/bash

echo "RoboRTS v0.5"
#set -o errexit 

gnome-terminal --window -e 'bash -c "roslaunch ${ROBORTS_PATH}/tools/stage/simulator.launch;exec bash"' \
sleep 1
gnome-terminal --window -e 'bash -c "cd ${ROBORTS_PATH};./build/modules/perception/localization/localization_node;exec bash"' \
sleep 1
gnome-terminal --window -e 'bash -c "cd ${ROBORTS_PATH};./build/modules/planning/global_planner/global_planner_node;exec bash"' \
sleep 1 
gnome-terminal --window -e 'bash -c "cd ${ROBORTS_PATH};./build/modules/planning/local_planner/local_planner_node;exec bash"' \
sleep 1
gnome-terminal --window -e 'bash -c "cd ${ROBORTS_PATH};./build/modules/decision/decision_node;exec bash"' \
