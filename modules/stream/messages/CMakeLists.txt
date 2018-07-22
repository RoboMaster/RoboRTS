project(messages)

add_action_files(
  DIRECTORY action
  FILES
  EnemyDirection.action
  example.action
  LocalPlanner.action
  GlobalPlanner.action
  ArmorDetection.action
  Localization.action
  BotGoal.action
)


add_message_files(
  DIRECTORY msg
  FILES
  EnemyPos.msg
  ShootState.msg
  Odometry.msg
  GimbalAngle.msg
  GameInfo.msg
  RfidInfo.msg
  ShootInfo.msg
  RobotHurtData.msg
  GoalTask.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GameBuffStatus.srv
  ChassisMode.srv
  GimbalMode.srv
  SelfCheck.srv
  CheckStatus.srv
  ShootModeControl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

set(msg_INCLUDE_DIRS "${messages_INCLUDE_DIRS}" CACHE  INTERNAL "messages Library location")
