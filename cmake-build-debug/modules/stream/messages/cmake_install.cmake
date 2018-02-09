# Install script for directory: /home/vic/Workspace/bug/RoboRTS/modules/stream/messages

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/messages/action" TYPE FILE FILES
    "/home/vic/Workspace/bug/RoboRTS/modules/stream/messages/action/example.action"
    "/home/vic/Workspace/bug/RoboRTS/modules/stream/messages/action/LocalPlanner.action"
    "/home/vic/Workspace/bug/RoboRTS/modules/stream/messages/action/GlobalPlanner.action"
    "/home/vic/Workspace/bug/RoboRTS/modules/stream/messages/action/ArmorDetection.action"
    "/home/vic/Workspace/bug/RoboRTS/modules/stream/messages/action/Localization.action"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/messages/msg" TYPE FILE FILES
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/exampleAction.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/exampleActionGoal.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/exampleActionResult.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/exampleActionFeedback.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/exampleGoal.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/exampleResult.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/exampleFeedback.msg"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/messages/msg" TYPE FILE FILES
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/LocalPlannerAction.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/LocalPlannerActionGoal.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/LocalPlannerActionResult.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/LocalPlannerActionFeedback.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/LocalPlannerGoal.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/LocalPlannerResult.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/LocalPlannerFeedback.msg"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/messages/msg" TYPE FILE FILES
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/GlobalPlannerAction.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/GlobalPlannerActionGoal.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/GlobalPlannerActionResult.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/GlobalPlannerActionFeedback.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/GlobalPlannerGoal.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/GlobalPlannerResult.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/GlobalPlannerFeedback.msg"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/messages/msg" TYPE FILE FILES
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/ArmorDetectionAction.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/ArmorDetectionActionGoal.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/ArmorDetectionActionResult.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/ArmorDetectionActionFeedback.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/ArmorDetectionGoal.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/ArmorDetectionResult.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/ArmorDetectionFeedback.msg"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/messages/msg" TYPE FILE FILES
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/LocalizationAction.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/LocalizationActionGoal.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/LocalizationActionResult.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/LocalizationActionFeedback.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/LocalizationGoal.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/LocalizationResult.msg"
    "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/messages/msg/LocalizationFeedback.msg"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/messages/msg" TYPE FILE FILES
    "/home/vic/Workspace/bug/RoboRTS/modules/stream/messages/msg/EnemyPos.msg"
    "/home/vic/Workspace/bug/RoboRTS/modules/stream/messages/msg/Odometry.msg"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/messages/cmake" TYPE FILE FILES "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/modules/stream/messages/catkin_generated/installspace/messages-msg-paths.cmake")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/include/messages")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/roseus/ros/messages")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/common-lisp/ros/messages")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/share/gennodejs/ros/messages")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/lib/python2.7/dist-packages/messages")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/vic/Workspace/bug/RoboRTS/cmake-build-debug/devel/lib/python2.7/dist-packages/messages")
endif()

