#!/bin/bash

function start_ros() {
    local MODULE_LOG="${LOG_DIR}/roscore.txt"
    check_process "simulator.launch"
    if [ "$?" -eq 0 ]; then
        echo "ROS has been launched, no need to start again!"
    else
#        source /opt/ros/kinetic/setup.bash
#        sleep 1s
        eval "nohup roslaunch ${ROOT_DIR}/tools/simulator/simulator.launch map:=${MAP} </dev/null >${MODULE_LOG} 2>&1 &"
        check_until_start "rviz"
        echo "ROS has been launched!"
    fi
}

function shutdown_ros() {
    pkill -f -SIGINT "ros"
    check_until_stop "ros"
    echo "ROS has been shutdown!"
}

function start() {

    local MODULE_PATH="$1"
    local MODULE_NAME="$(basename ${MODULE_PATH})"
    local MODULE_LOG="${LOG_DIR}/${MODULE_NAME}.txt"
    check_process "${MODULE_NAME}"
    if [ "$?" -eq 0 ]; then
        echo "${MODULE_NAME} has been started, no need to start again!"
    else
        eval "nohup ${MODULE_PATH} </dev/null >${MODULE_LOG} 2>&1 &"
        check_until_start "${MODULE_NAME}"
    fi
}

function shutdown() {
    local MODULE_PATH="$1"
    local MODULE_NAME="$(basename ${MODULE_PATH})"
    pkill -9 -f "${MODULE_NAME}"
    check_until_stop "${MODULE_NAME}"
}

function check_until_stop() {
    local PROCESS="$1"
    local DELAY="$2"
    if [ -z "${DELAY}" ];then
        DELAY="1s"
    fi
    check_process "${PROCESS}"
    while [ ! "$?" -eq 1 ]; do
       sleep "${DELAY}"
       check_process "${PROCESS}"
    done
}

function check_until_start() {
    local PROCESS="$1"
    local DELAY="$2"
    if [ -z "${DELAY}" ];then
        DELAY="1s"
    fi
    check_process "${PROCESS}"
    while [ "$?" -eq 1 ]; do
       sleep "${DELAY}"
       check_process "${PROCESS}"
    done
}

function check_process() {
   local PROCESS="$1"
   local PROCESS_NUM="$(pgrep -c -f "${PROCESS}")"
   if [ "${PROCESS_NUM}" -eq 0 ]; then
	return 1
   else
	return 0
   fi
}

function main(){

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$(dirname $( dirname "${SCRIPT_DIR}"))"
cd ${ROOT_DIR}
mkdir -p /tmp/RoboRTS/log
LOG_DIR="/tmp/RoboRTS/log"
echo "RoboRTS Version 0.5"
echo "Path: ${ROOT_DIR}"

LOCALIZATION_MODULE="build/modules/perception/localization/localization_node"
GLOBAL_PLANNING_MODULE="build/modules/planning/global_planner/global_planner_node"
LOCAL_PLANNING_MODULE="build/modules/planning/local_planner/local_planner_node"
DECISION_MODULE="build/modules/decision/decision_node"

MODULES=(${LOCALIZATION_MODULE} \
        ${GLOBAL_PLANNING_MODULE} \
        ${LOCAL_PLANNING_MODULE} \
        ${DECISION_MODULE})

start_ros

for MODULE in ${MODULES[@]}
do
    start ${MODULE}
done

local CLOSE=0
while [ "${CLOSE}" -eq 0 ]; do
    read -n1 -p "Close[Y/N]?" ANS
    case $ANS in
    Y | y)
          CLOSE=1;;
    N | n)
          ;;
    *)
         ;;
    esac
done

shutdown_ros
for MODULE in ${MODULES[@]}
do
    shutdown ${MODULE}
done
}

MAP="rm"
INPUT_MAP="$1"
if [ ! -z "$INPUT_MAP" ]
then
  if [ -e "${ROOT_DIR}/tools/map/${INPUT_MAP}.yaml" ]
  then
    MAP="$INPUT_MAP"
  fi
fi

main