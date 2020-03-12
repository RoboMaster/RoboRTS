cmake_minimum_required(VERSION 2.8.3)
project(externals)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-w -Wno-dev)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES externals
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
  DEPENDS system_lib
)

add_executable(${PROJECT_NAME}_node
	UartDemo.cpp
)

target_link_libraries(${PROJECT_NAME}_node
PRIVATE
${catkin_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}_node
	PRIVATE
	${catkin_INCLUDE_DIRS}
)
