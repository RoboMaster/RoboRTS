project(costmap)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
        CostmapProtoSrcs CostmapProtoHeaders
        ${ProtoFiles})

add_library(costmap SHARED
        costmap_math.cpp
        layer.cpp
        costmap_2d.cpp
        costmap_layer.cpp
        layered_costmap.cpp
        costmap_interface.cpp
        observation_buffer.cpp
        obstacle_layer.cpp
        footprint.cpp
        static_layer.cpp
        obstacle_layer.cpp
        inflation_layer.cpp
        ${CostmapProtoSrcs}
        ${CostmapProtoHeaders}
)
add_library(perception::map::costmap ALIAS costmap)
target_include_directories(costmap
        PUBLIC
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)
target_link_libraries(costmap
        PUBLIC
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${GLOG_LIBRARY}
        common::io
        )

add_executable(test_costmap test_costmap.cpp)
target_link_libraries(test_costmap
        costmap
        ${catkin_LIBRARIES}
        ${GLOG_LIBRARY}
        )
