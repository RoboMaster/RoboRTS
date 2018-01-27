project(serial_comm)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto SerialProtoSrc SerialProtoHds ${ProtoFiles})

#add_executable(serial_read_node
#        ${SerialProtoSrc}
#        ${SerialProtoHds}
#        com_read_node.cpp
#        serial_comm_read.cpp
#        serial_comm.cpp
#        )
#
#target_include_directories(serial_read_node
#        PUBLIC
#        ${CMAKE_CURRENT_SOURCE_DIR}
#        ${Boost_INCLUE_DIRS}
#        )
#
#target_link_libraries(serial_read_node
#        PRIVATE
#        common::rrts
#        common::io
#
#        ${catkin_LIBRARIES}
#        ${PROTOBUF_LIBRARIES}
#        ${GLOG_LIBRARY}
#        ${Boost_LIBRARIES}
#        )
#
#add_executable(serial_write_node
#        ${SerialProtoSrc}
#        ${SerialProtoHds}
#        serial_comm.cpp
#        com_write_node.cpp
#        serial_comm_write.cpp
#        )
#
#target_include_directories(serial_write_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
#
#target_link_libraries(serial_write_node
#        PRIVATE
#        common::rrts
#        common::io
#
#        ${catkin_LIBRARIES}
#        ${PROTOBUF_LIBRARIES}
#        ${GLOG_LIBRARY}
#        )

add_executable(serial_com_node serial_com_node.cpp ${SerialProtoSrc})
target_include_directories(serial_com_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(serial_com_node
        PRIVATE
        common::rrts
        common::io
        ${catkin_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
        ${GLOG_LIBRARY}
)