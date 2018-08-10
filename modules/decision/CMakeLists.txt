project(decision)
add_subdirectory(behavior_tree)


add_executable(ICRA_decision_node
  ICRA_decision/decision_node.cpp
  ICRA_decision/goal_factory.h
  )

target_link_libraries(ICRA_decision_node
  PRIVATE
  common::log
  behavior_tree
  perception::map::costmap
  ${catkin_LIBRARIES}
  )


add_executable(test_decision_node
  decision_test.cpp
  )

target_link_libraries(test_decision_node
  PRIVATE
  common::io
  common::log
  ${catkin_LIBRARIES}
  )

