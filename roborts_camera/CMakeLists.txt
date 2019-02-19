project(roborts_camera)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-w -Wno-dev)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  actionlib
  roborts_common
  cv_bridge
  image_transport
  )

find_package(ProtoBuf REQUIRED)
find_package(OpenCV 3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
)

add_subdirectory(uvc)

#camera parameter
file(GLOB CameraParamProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/camera_param.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  CameraParamProtoSrc
  CameraParamProtoHds
  ${CameraParamProtoFiles}
  )

#camera param
add_library(roborts_camera_param
  SHARED
  ${CameraParamProtoSrc}
  ${CameraParamProtoHds}
  camera_param.cpp
)

target_link_libraries(roborts_camera_param
  PUBLIC
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

target_include_directories(roborts_camera_param
  PUBLIC
  ${catkin_INCLUDE_DIRS}
)

#camera_node
add_executable(${PROJECT_NAME}_node
  camera_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
  driver::uvc_driver
  roborts_camera_param
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}_node
  PRIVATE
  ${OpenCV_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

#cap_img

add_executable(image_capture_test
  test/image_capture.cpp
)

target_link_libraries(image_capture_test
  PRIVATE
  driver::uvc_driver
  roborts_camera_param
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_include_directories(image_capture_test
  PRIVATE
  ${OpenCV_INCLUDE_DIRECTORIES}
)
