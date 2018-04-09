project(camera_node)

add_subdirectory(uvc)

#camera parameter
file(GLOB CameraParamProtoFiles "proto/camera_param.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto CameraParamProtoSrc CameraParamProtoHds ${CameraParamProtoFiles})

add_library(camera_param
  SHARED
  ${CameraParamProtoSrc}
  ${CameraParamProtoHds}
  camera_param.cpp
)

add_library(driver::camera_param ALIAS camera_param)

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
  camera_base.cpp
  camera_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
  driver::uvc_driver
  driver::camera_param

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
