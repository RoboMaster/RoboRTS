project(amcl)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto AmclProtoSrc AmclProtoHds ${ProtoFiles})


add_library(amcl
        ${AmclProtoSrc}
        ${AmclProtoHds}
        amcl.cpp
        amcl.h
        ${CMAKE_CURRENT_SOURCE_DIR}/map/amcl_map.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/math/math.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter_gaussian_pdf.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter_kdtree.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter_sample.h
        ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensor_base.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensor_laser.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensor_odom.cpp)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        common::main_interface
        common::log
        common::io
        common::rrts
        ${EIGEN3_LIBRARIES}
        ${catkin_LIBRARIES}
        ${GLOG_LIBRARY}
        )

add_library(localization::amcl ALIAS amcl)
