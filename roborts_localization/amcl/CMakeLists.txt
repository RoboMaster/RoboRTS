project(amcl)

#find_package(catkin REQUIRED COMPONENTS
#        roscpp
#        tf
#        nav_msgs
#        message_filters
#        sensor_msgs
#        )

add_library(amcl
        amcl.cpp
        amcl.h
        amcl_config.h
        ${CMAKE_CURRENT_SOURCE_DIR}/map/amcl_map.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter_gaussian_pdf.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter_kdtree.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter_sample.h
        ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensor_laser.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensor_odom.cpp)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        ${EIGEN3_LIBRARIES}
        ${catkin_LIBRARIES}
        ${GLOG_LIBRARY}
        )

add_library(localization::amcl ALIAS amcl)
