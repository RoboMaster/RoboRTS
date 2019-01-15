project(detection_util)

add_library(cv_toolbox INTERFACE)
target_sources(cv_toolbox INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(cv_toolbox INTERFACE ${catkin_LIBRARIES}
  )
target_include_directories(cv_toolbox
  INTERFACE
  ${catkin_INCLUDE_DIRS}
  )
add_library(detection::tool ALIAS cv_toolbox)