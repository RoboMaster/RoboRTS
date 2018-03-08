#!/bin/bash

echo "RoboRTS v0.5"

function ProcessDetection() {
    ps_out=`ps -ef | grep $1 | grep -v 'grep' | grep -v $0`
    result=$(echo $ps_out | grep "$1")
    if [[ "$result" != "" ]];then
        return 1
    else
        return 0
    fi
}
MAP="icra"
INPUT_MAP="$1"
if [ ! -z "$INPUT_MAP" ]
then
  if [ -e "${ROBORTS_PATH}/tools/map/"$INPUT_MAP".yaml" ]
  then 
    MAP="$INPUT_MAP"
  fi
fi  
echo "Open map ${MAP}"
roslaunch ${ROBORTS_PATH}/tools/example/example.launch map:=${MAP}&
ProcessDetection example.launch

rst=$?
while [ "$rst" = "0" ]; do
    ProcessDetection example.launch
    rst=$?
    sleep 1
done

cd ${ROBORTS_PATH}
./build/modules/driver/serial/serial_com_node &
./build/modules/stream/tf_tree/tf_tree &
./build/modules/perception/localization/localization_node &
./build/modules/planning/global_planner/global_planner_node &
./build/modules/planning/local_planner/local_planner_node &
./build/modules/decision/decision_node &
echo "finish!"
