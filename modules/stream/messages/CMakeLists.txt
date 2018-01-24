project(messages)

add_action_files(
  DIRECTORY action
  FILES
  example.action
  LocalPlanner.action
  GlobalPlanner.action
  ArmorDetection.action
  Localization.action
)

add_message_files(
  DIRECTORY msg
  FILES
  EnemyPos.msg
  Odometry.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

set(msg_INCLUDE_DIRS "${messages_INCLUDE_DIRS}" CACHE  INTERNAL "messages Library location")
