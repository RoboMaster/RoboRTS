project(uvc_driver)

#camera parameter
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto UVCDriverProtoSrc UVCDriverProtoHds ${ProtoFiles})

#uvc_driver
add_library(uvc_driver
  SHARED
  uvc_driver.cpp
)

add_library(driver::uvc_driver ALIAS uvc_driver)

target_link_libraries(uvc_driver
  PRIVATE
  driver::camera_param

  common::main_interface
  common::log
  common::rrts
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARY}
  ${OpenCV_LIBRARIES}
)

target_include_directories(uvc_driver
  PRIVATE
  ${OpenCV_INCLUDE_DIRECTORIES}
)
