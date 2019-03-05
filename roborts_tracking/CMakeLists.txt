cmake_minimum_required(VERSION 3.0)
project(roborts_tracking)
set(CMAKE_BUILD_TYPE Release)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  roborts_msgs
  )

if(NOT WIN32)
  ADD_DEFINITIONS("-std=c++11")
endif(NOT WIN32)

catkin_package()

add_library(kcf_tracker STATIC
  KCFcpp/src/fhog.cpp
  KCFcpp/src/kcftracker.cpp)
target_link_libraries(kcf_tracker ${OpenCV_LIBRARIES})
target_include_directories(kcf_tracker PRIVATE KCFcpp/src)

add_executable(roborts_tracking_test
  tracking_test.cpp
  tracking_utility.cpp)
target_link_libraries(roborts_tracking_test
  kcf_tracker
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES})
target_include_directories(roborts_tracking_test
  PUBLIC
  ${catkin_INCLUDE_DIRS})
add_dependencies(roborts_tracking_test
  roborts_msgs_generate_messages)

