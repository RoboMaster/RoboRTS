project(common)

#common::io
add_library(io INTERFACE)
add_library(common::io ALIAS io)
target_sources(io INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/io.h)
target_link_libraries(io INTERFACE ${PROTOBUF_LIBRARIES})

#common::log
add_library(log INTERFACE)
add_library(common::log ALIAS log)
target_sources(log INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/log.h)
target_link_libraries(log INTERFACE ${GLOG_LIBRARY})

#common::main_interface
add_library(main_interface INTERFACE)
add_library(common::main_interface  ALIAS main_interface)
target_sources(main_interface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/main_interface.h)
target_link_libraries(main_interface INTERFACE ${catkin_LIBRARIES} ${GLOG_LIBRARY})

#common::rrts
add_library(rrts SHARED rrts.cpp)
add_library(common::rrts ALIAS  rrts)
target_include_directories(rrts PUBLIC ${catkin_INCLUDE_DIRS})

# This project indicates the design structure of our platform, only for testing purpose and no practical significance.
#add_subdirectory(example)
