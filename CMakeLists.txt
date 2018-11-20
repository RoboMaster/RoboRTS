project(roborts)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_BUILD_TYPE Release) 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  cv_bridge
  image_transport
# laser
  laser_geometry
  pcl_conversions
  pcl_ros
# message
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
# actionlib and message generation
  actionlib
  message_generation
  interactive_markers
)


add_action_files(
  DIRECTORY modules/stream/messages/action
  FILES
  EnemyDirection.action
  example.action
  LocalPlanner.action
  GlobalPlanner.action
  ArmorDetection.action
  InfantryInfo.action
  Localization.action
  BotGoal.action
)


add_message_files(
  DIRECTORY modules/stream/messages/msg
  FILES
  EnemyPos.msg
  ShootState.msg
  Odometry.msg
  GimbalAngle.msg
  GameInfo.msg
  RfidInfo.msg
  ShootInfo.msg
  RobotHurtData.msg
  GoalTask.msg
)

add_service_files(
  DIRECTORY modules/stream/messages/srv
  FILES
  GameBuffStatus.srv
  ChassisMode.srv
  GimbalMode.srv
  SelfCheck.srv
  CheckStatus.srv
  ShootModeControl.srv
)



find_package(Boost REQUIRED COMPONENTS
  system
  thread
)

find_package(OpenCV 3 REQUIRED)
find_package(G2O REQUIRED)
find_package(SUITESPARSE REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Glog REQUIRED)
find_package(ProtoBuf REQUIRED)

add_subdirectory(modules/stream/messages)
list(APPEND catkin_INCLUDE_DIRS ${msg_INCLUDE_DIRS})
message(STATUS ${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${catkin_INCLUDE_DIRS})
catkin_package()
add_subdirectory(common)
add_subdirectory(modules)
