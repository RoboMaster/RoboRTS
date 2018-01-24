project (tf_tree)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto ProtoSources ProtoHeaders ${ProtoFiles})

add_executable(${PROJECT_NAME}
  ${ProtoSources}
  ${ProtoHeaders}
  tf_tree.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    common::main_interface
    common::rrts
    ${PROTOBUF_LIBRARIES}
    ${catkin_LIBRARIES}
    ${GLOG_LIBRARY}
    ${Boost_LIBRARIES}
)

