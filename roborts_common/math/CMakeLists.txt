cmake_minimum_required(VERSION 3.1)
project(roborts_math)

#common::math
add_library(roborts_math INTERFACE

)

target_sources(roborts_math INTERFACE
  geometry.h
  math.h
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  )

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARY}
  )
