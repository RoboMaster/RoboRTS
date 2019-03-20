project(roborts_base)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)
find_package(Threads REQUIRED)
find_package(Glog REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  roborts_msgs
  )

catkin_package()

add_executable(roborts_base_node
  roborts_base_node.cpp
  chassis/chassis.cpp
  gimbal/gimbal.cpp
  referee_system/referee_system.cpp
  roborts_sdk/dispatch/execution.cpp
  roborts_sdk/dispatch/handle.cpp
  roborts_sdk/protocol/protocol.cpp
  roborts_sdk/hardware/serial_device.cpp
  )
target_link_libraries(roborts_base_node PUBLIC
  Threads::Threads
  ${GLOG_LIBRARY}
  ${catkin_LIBRARIES})
target_include_directories(roborts_base_node PUBLIC
  ${catkin_INCLUDE_DIRS})
add_dependencies(roborts_base_node roborts_msgs_generate_messages)

