project(localization)
add_subdirectory(amcl)

file(GLOB LocalizationProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

rrts_protobuf_generate_cpp(
        ${CMAKE_CURRENT_SOURCE_DIR}/proto
        LocalizationProtoSrcs
        LocalizationProtoHeaders
        ${LocalizationProtoFiles})

add_executable(localization_node
        ${LocalizationProtoSrcs}
        ${LocalizationProtoHeaders}
        localization_node.cpp
        localization_node.h
        )

target_link_libraries(localization_node
        PUBLIC
        localization::amcl
        common::main_interface
        common::log
        common::io
        common::rrts
        ${EIGEN3_LIBRARIES}
        ${catkin_LIBRARIES}
        ${GLOG_LIBRARY}
        )

#localization node client test
add_executable(localization_client
  localization_client.cpp
  )

target_link_libraries(localization_client
  PRIVATE
  common::log
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARY}
  )


