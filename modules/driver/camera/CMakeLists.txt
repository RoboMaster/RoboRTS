project(camera_node)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" )
    add_subdirectory(mercure)
    ADD_DEFINITIONS(-DAARCH64)
else()
    message("Warning: The mercure driver only support arm64")
endif()

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

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" )
target_link_libraries(${PROJECT_NAME}
  PRIVATE
  driver::mercure
)
endif()

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

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" )
target_link_libraries(${PROJECT_NAME}
  PRIVATE
  driver::mercure
)
endif()

target_include_directories(${PROJECT_NAME}
  PRIVATE
  ${OpenCV_INCLUDE_DIRECTORIES}
)
