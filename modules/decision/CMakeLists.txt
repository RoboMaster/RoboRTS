project(decision)
add_subdirectory(behavior_tree)


add_executable(ICRA_decision_node
  ICRA_decision/decision_node.cpp
  ICRA_decision/goal_factory.h
  ICRA_decision/action_behavior.h
  )

target_link_libraries(ICRA_decision_node
  PRIVATE
  common::log
  behavior_tree
  perception::map::costmap
  ${catkin_LIBRARIES}
  )


add_executable(test_decision_node
        test_decision/decision_node.cpp
        )
target_link_libraries(test_decision_node
        PRIVATE
        common::log
        perception::map::costmap
        ${catkin_LIBRARIES}
        )

add_executable(${PROJECT_NAME}_test
  decision_test.cpp
  )

target_link_libraries(${PROJECT_NAME}_test
  PRIVATE
  common::io
  common::log
  perception::map::costmap
  ${catkin_LIBRARIES}
  )

