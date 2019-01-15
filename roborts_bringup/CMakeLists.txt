project(roborts_bringup)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roborts_base
  roborts_common
  roborts_localization
  roborts_costmap
  roborts_msgs
  roborts_planning
  )

catkin_package()


