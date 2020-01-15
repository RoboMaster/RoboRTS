project(lightBlob_test)

add_library(lightBlob_test
  SHARED
  lightBlob_test.cpp
  )

add_library(detection::lightBlob_test ALIAS lightBlob_test)

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