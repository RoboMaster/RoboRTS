#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$(dirname $( dirname "${SCRIPT_DIR}"))"
cd ${ROOT_DIR}
mkdir -p /tmp/RoboRTS/log
LOG_DIR="/tmp/RoboRTS/log"
echo "RoboRTS Version 0.5"
echo "Path: ${ROOT_DIR}"


SERIAL_MODULE="build/modules/driver/serial/serial_com_node"
TF_MODULE="build/modules/stream/tf_tree/tf_tree"
LOCALIZATION_MODULE="build/modules/perception/localization/localization_node"
GLOBAL_PLANNING_MODULE="build/modules/planning/global_planner/global_planner_node"
LOCAL_PLANNING_MODULE="build/modules/planning/local_planner/local_planner_node"
DECISION_MODULE="build/modules/decision/decision_node"

MAP="icra"
SIMULATION=1

function start_ros() {
    local MODULE_LOG="${LOG_DIR}/roscore.txt"
    check_process "simulator.launch"
    if [ "$?" -eq 0 ]; then
        echo "ROS has been launched, restart right now!"
        shutdown_ros
        start_ros
    else
#        source /opt/ros/kinetic/setup.bash
#        sleep 1s
        if [ "$SIMULATION" -eq 0 ]; then
        eval "nohup roslaunch ${ROOT_DIR}/tools/example/example.launch map:=${MAP} </dev/null >${MODULE_LOG} 2>&1 &"
        check_until_start "rplidar_ros"
        else
        eval "nohup roslaunch ${ROOT_DIR}/tools/simulator/simulator.launch map:=${MAP} </dev/null >${MODULE_LOG} 2>&1 &"
        check_until_start "rviz"
        fi

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
        echo "${MODULE_NAME} has been started, restart right now!"
        shutdown "${MODULE_PATH}"
        start "${MODULE_PATH}"
    else
        eval "nohup ${MODULE_PATH} </dev/null >${MODULE_LOG} 2>&1 &"
        check_until_start "${MODULE_NAME}"
        echo "${MODULE_NAME} has been launched!"
    fi
}

function shutdown() {
    local MODULE_PATH="$1"
    local MODULE_NAME="$(basename ${MODULE_PATH})"
    pkill -9 -f "${MODULE_NAME}"
    check_until_stop "${MODULE_NAME}"
    echo "${MODULE_NAME} has been shutdown!"
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

function run_all(){
if [ "${SIMULATION}" -eq 1 ]; then
    echo "Simulation Mode"
	MODULES=(${LOCALIZATION_MODULE} \
        	${GLOBAL_PLANNING_MODULE} \
        	${LOCAL_PLANNING_MODULE} \
        	${DECISION_MODULE})
else
    echo "Test Mode"
	MODULES=(${SERIAL_MODULE} \
	        ${TF_MODULE} \
	        ${LOCALIZATION_MODULE} \
        	${GLOBAL_PLANNING_MODULE} \
        	${LOCAL_PLANNING_MODULE} \
        	${DECISION_MODULE})
fi

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

function help(){
  echo "Usage:
  $0 [COMMAND]"
  echo "COMMAND:
  --help -h
        Show help message
  --simulation or -s
        Run in stage simulator
  --map or -m
        Set the map
  "
}

function default_config(){
MAP="icra"
SIMULATION=1
}

if [[ $# -eq 0  ]];then
    default_config
fi

ARGS=`getopt -o "s,m:,h" -l "simulation,map:,help" -- "$@"`
if [ $? != 0 ] ; then 
    help
    exit 1
fi
eval set -- "${ARGS}"



while true;
do
    case "$1" in
        "--simulation"|"-s")
            SIMULATION=1
            shift
            ;;
        "--map"|"-m")
            INPUT_MAP="$2"
            if [ ! -z "$INPUT_MAP" ]; then
              if [ -e "${ROOT_DIR}/tools/map/${INPUT_MAP}.yaml" ]; then
                MAP="$INPUT_MAP"
	      else
		echo "Map ${INPUT_MAP} is not found, ${ROOT_DIR}/tools/map/${INPUT_MAP}.yaml does not exist."
                exit 1
	      fi
            fi
            echo "Load map: ${MAP}"
            shift 2
            ;;
        "--help"|"-h")
            help
            exit 0
            ;;
        --)
            shift
            break
            ;;
        *)
            echo "Internal error!"
            exit 1
            ;;
    esac
done
if [ ! "$#" -eq 0 ]; then
    for arg do
        echo "Arg $arg is invalid."
	help
	exit 1
    done
else
    run_all
fi 
