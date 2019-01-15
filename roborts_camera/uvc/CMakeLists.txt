project(uvc_driver)

#uvc_driver
add_library(uvc_driver
  SHARED
  uvc_driver.cpp
)

target_link_libraries(uvc_driver
  PRIVATE
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_include_directories(uvc_driver
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
)

add_library(driver::uvc_driver ALIAS uvc_driver)
