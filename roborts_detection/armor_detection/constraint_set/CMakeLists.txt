project(constraint_set)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  ConstraintSetProtoSrc
  ConstraintSetProtoHds
  ${ProtoFiles}
  )

add_library(constraint_set
  SHARED
  ${ConstraintSetProtoSrc}
  ${ConstraintSetProtoHds}
  constraint_set.cpp
  )

add_library(detection::constraint_set ALIAS constraint_set)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  detection::tool
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
