project(mercure)

add_library(${PROJECT_NAME}
  SHARED
  fps.cpp
  mercure_driver.cpp
)

add_library(driver::mercure ALIAS mercure)

target_link_libraries(${PROJECT_NAME}
  libgxiapi.so
  pthread
  ${OpenCV_LIBRARIES}
)
