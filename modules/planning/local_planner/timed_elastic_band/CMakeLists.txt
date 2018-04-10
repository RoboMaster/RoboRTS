cmake_minimum_required(VERSION 3.0)
project(timed_elastic_band)

set(EXTERNAL_INCLUDE_DIRS
  ${EIGEN3_INCLUDE_DIR}
  ${SUITESPARSE_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  )
set(EXTERNAL_LIBS
  ${SUITESPARSE_LIBRARIES}
  ${G2O_LIBRARIES}
  )


include_directories(${EXTERNAL_INCLUDE_DIRS} )
aux_source_directory(.  SRC_LIST)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto timed_elastic_bandProtoSrc timed_elastic_bandProtoHds ${ProtoFiles})

add_library(${PROJECT_NAME}
  SHARED
  ${timed_elastic_bandProtoSrc}
  ${timed_elastic_bandProtoHds}
  ${SRC_LIST}
  )

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  common::log
  common::io
  perception::map::costmap
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARY}
  ${EXTERNAL_LIBS}
  )
add_library(planning::timed_elastic_band ALIAS timed_elastic_band)
