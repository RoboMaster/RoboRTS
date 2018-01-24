project(camera_driver)

#camera parameter
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto CameraDriverProtoSrc CameraDriverProtoHds ${ProtoFiles})

add_library(camera_param
  SHARED
  ${CameraDriverProtoSrc}
  ${CameraDriverProtoHds}
  camera_param.cpp
  )

add_library(drivers::camera_param ALIAS camera_param)

target_link_libraries(camera_param
  PUBLIC
  common::io
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

target_include_directories(camera_param
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  )

#camera_node
add_executable(${PROJECT_NAME}
  camera_node.cpp
  )

target_link_libraries(${PROJECT_NAME}
  PRIVATE
  drivers::camera_param
  common::main_interface
  common::log
  common::rrts
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARY}
  ${OpenCV_LIBRARIES}
  )

target_include_directories(${PROJECT_NAME}
  PRIVATE
  ${OpenCV_INCLUDE_DIRECTORIES}
  )
