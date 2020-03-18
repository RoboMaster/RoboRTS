project(mvc_driver)

#mvc_driver
add_library(mvc_driver
  SHARED
  mvc_driver.cpp
)

target_link_libraries(mvc_driver
  PRIVATE
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_include_directories(mvc_driver
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
)

add_library(driver::mvc_driver ALIAS mvc_driver)
