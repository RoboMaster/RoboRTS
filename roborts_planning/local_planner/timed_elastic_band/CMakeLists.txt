cmake_minimum_required(VERSION 3.1)
project(timed_elastic_band)


aux_source_directory(src/.  SRC_LIST)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/proto/*.proto")
rrts_protobuf_generate_cpp(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/proto
  timed_elastic_bandProtoSrc
  timed_elastic_bandProtoHds
  ${ProtoFiles})

add_library(${PROJECT_NAME}
  SHARED
  ${timed_elastic_bandProtoSrc}
  ${timed_elastic_bandProtoHds}
  ${SRC_LIST}
  )

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${EXTERNAL_LIBS}
  )
add_library(planning::timed_elastic_band ALIAS timed_elastic_band)
