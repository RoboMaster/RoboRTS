#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "messages/LocalizationAction.h"
#include <actionlib/client/terminal_state.h>

#include "common/log.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "localization_client");

  // create the action client
  actionlib::SimpleActionClient<messages::LocalizationAction> ac("localization_node_action", true);
  //messages::ArmorDetectionResult node_result;
  LOG_INFO<<"Waiting for localization node start.";
  ac.waitForServer();
  LOG_INFO<<"Localization node Started.";
  messages::LocalizationGoal goal;

  char command = '0';

  while (command != '4') {
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "*********************************please send a command********************************" << std::endl;
    std::cout << "1: start the action" << std::endl <<
                 "2: pause the action" << std::endl <<
                 "3: stop  the action" << std::endl <<
                 "4: exit the program" << std::endl;
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != '1' && command != '2' && command != '3' && command != '4') {
      std::cout << "please input 1, 2 or 3!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

    switch (command) {
      //start thread.
      case '1':
        goal.command = 1;
        LOG_INFO << "Localization node will be started.";
        ac.sendGoal(goal);
        break;
        //pause thread.
      case '2':
        goal.command = 2;
        LOG_INFO << "Localization node will be paused.";
        ac.sendGoal(goal);
        //stop thread.
      case '3':
        goal.command = 3;
        LOG_INFO<<"Localization node will be closed.";
        ac.cancelGoal();
        break;
      default:
        break;
    }
  }
  return 0;
}
