project(example)

add_executable(example_node
  example_node.cpp
  example_algorithm.cpp
  )
target_link_libraries(example_node
  PRIVATE
  common::main_interface
  common::rrts
  common::log
)

add_executable(example_client
  example_client.cpp)
target_link_libraries(example_client
  PRIVATE
  common::log
  ${catkin_LIBRARIES}
)
