#include <ros/ros.h>
#include <memory>
#include <atomic>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "executor/chassis_executor.h"
#include "executor/gimbal_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"

namespace {
  std::atomic<char> g_command{'0'};
  std::atomic<bool> g_running{true};
}

void CommandInputThread();

int main(int argc, char **argv) {
  ros::init(argc, argv, "behavior_test_node");

  std::string full_path = ros::package::getPath("roborts_decision")
                         + "/config/decision.prototxt";

  // Create executors with unique_ptr for clear ownership
  auto chassis_executor = std::make_unique<roborts_decision::ChassisExecutor>();
  auto blackboard = std::make_unique<roborts_decision::Blackboard>(full_path);

  // Get raw pointers for behavior constructors (behaviors don't own these)
  auto* chassis_executor_ptr = chassis_executor.get();
  auto* blackboard_ptr = blackboard.get();

  // Initialize all behaviors
  roborts_decision::BackBootAreaBehavior back_boot_area_behavior(
      chassis_executor_ptr, blackboard_ptr, full_path);
  roborts_decision::ChaseBehavior chase_behavior(
      chassis_executor_ptr, blackboard_ptr, full_path);
  roborts_decision::SearchBehavior search_behavior(
      chassis_executor_ptr, blackboard_ptr, full_path);
  roborts_decision::EscapeBehavior escape_behavior(
      chassis_executor_ptr, blackboard_ptr, full_path);
  roborts_decision::PatrolBehavior patrol_behavior(
      chassis_executor_ptr, blackboard_ptr, full_path);
  roborts_decision::GoalBehavior goal_behavior(
      chassis_executor_ptr, blackboard_ptr);

  // Start command input thread
  auto command_thread = std::thread(CommandInputThread);

  ros::Rate rate(10);
  while (ros::ok() && g_running.load()) {
    ros::spinOnce();

    const char cmd = g_command.load();
    switch (cmd) {
      case '1':
        back_boot_area_behavior.Run();
        break;
      case '2':
        patrol_behavior.Run();
        break;
      case '3':
        chase_behavior.Run();
        break;
      case '4':
        search_behavior.Run();
        break;
      case '5':
        escape_behavior.Run();
        break;
      case '6':
        goal_behavior.Run();
        break;
      case 27:  // ESC key
        g_running.store(false);
        break;
      default:
        break;
    }

    rate.sleep();
  }

  // Clean shutdown
  g_running.store(false);
  if (command_thread.joinable()) {
    command_thread.join();
  }

  ROS_INFO("Behavior test node shutdown complete.");
  return 0;
}

void CommandInputThread() {
  while (g_running.load()) {
    std::cout << "\n"
              << "╔══════════════════════════════════════════════════════╗\n"
              << "║          RoboRTS Behavior Test Controller           ║\n"
              << "╠══════════════════════════════════════════════════════╣\n"
              << "║  1: Back to boot area    4: Search behavior         ║\n"
              << "║  2: Patrol behavior      5: Escape behavior         ║\n"
              << "║  3: Chase behavior       6: Goal behavior           ║\n"
              << "║                        ESC: Exit program            ║\n"
              << "╚══════════════════════════════════════════════════════╝\n"
              << "> " << std::flush;

    char input;
    std::cin >> input;

    if (input >= '1' && input <= '6') {
      g_command.store(input);
    } else if (input == 'q' || input == 'Q') {
      g_command.store(27);
      g_running.store(false);
      break;
    } else {
      std::cout << "Invalid command. Please try again." << std::endl;
    }
  }
}
