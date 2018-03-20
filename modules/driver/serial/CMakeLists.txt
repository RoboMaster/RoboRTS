project(serial_comm)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto SerialProtoSrc SerialProtoHds ${ProtoFiles})

add_executable(serial_com_node serial_com_node.cpp ${SerialProtoSrc} ${SerialProtoHds})
target_include_directories(serial_com_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(serial_com_node
        PRIVATE
        common::rrts
        common::io
        ${catkin_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
        ${GLOG_LIBRARY}
        )