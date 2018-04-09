project(constraint_set)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto ConstraintSetProtoSrc ConstraintSetProtoHds ${ProtoFiles})

add_library(constraint_set
  SHARED
    ${ConstraintSetProtoSrc}
    ${ConstraintSetProtoHds}
    constraint_set.cpp
  )

add_library(detection::constraint_set ALIAS constraint_set)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    driver::camera_param
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )

target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
