/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "messages/exampleAction.h"

#include "common/log.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_client");
  actionlib::SimpleActionClient<messages::exampleAction> ac("test_action", true);
  messages::exampleResult node_result;
  LOG_INFO<<"Waiting for action server to start.";
  ac.waitForServer();

  messages::exampleGoal goal;

  int command = 0;

  while (command != 3) {
    std::cout<<"Send a command: "<<std::endl;
    std::cout<<"1: send the action"<<std::endl;
    std::cout<<"2: cancel the action"<<std::endl;
    std::cout<<"3: exit the program"<<std::endl;
    std::cin >> command;

    switch (command) {
      case 1:
        LOG_INFO<<"Send the action!";
        ac.sendGoal(goal);
        send_action = true;
        node_result = *(ac.getResult());
        LOG_INFO<<"Action finished, status: "<< node_result.error_code;
        break;
      case 2:
        LOG_INFO<<"Cancel the action!";
        ac.cancelGoal();
        send_action = false;
        break;
      default:
        break;
    }
  }
  return 0;
}
