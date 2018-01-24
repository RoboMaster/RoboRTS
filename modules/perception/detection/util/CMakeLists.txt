project(detection_util)

#common::log
add_library(cv_toolbox INTERFACE)
add_library(detection::image_buffer ALIAS cv_toolbox)
target_sources(cv_toolbox INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/cv_toolbox.h)
