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
//#include "common/log.h"#include "common/log.h"

#include "messages/exampleAction.h"


int main(int argc, char **argv) {

  ros::init(argc, argv, "test_client");
  actionlib::SimpleActionClient<messages::exampleAction> ac("test_action", true);
  messages::exampleResult node_result;
  std::cout<<"Waiting for action server to start."<<std::endl;
  // wait for the action server to start
  ac.waitForServer();  // will wait for infinite time

  messages::exampleGoal goal;

  int command = 0;
  bool send_action = false;

  while (command != 3) {
    std::cout<<"Send a command: 1:start the action | 2:stop the action | 3:exit the program"<<std::endl;
    std::cin >> command;

    switch (command) {
      case 1:
        if (!send_action) {
          std::cout<<"I am running the request"<<std::endl;
          ac.sendGoal(goal);
          send_action = true;
          node_result = *(ac.getResult());
          std::cout<<"Action finished, status: "<< node_result.error_code<<std::endl;
        } else {
          std::cout<<"I am re-running the request"<<std::endl;
//          ac.cancelGoal();
          ac.sendGoal(goal);
          std::cout<<"Action finished, status: "<< node_result.error_code<<std::endl;
        }
        break;
      case 2:
        std::cout<<"I am cancelling the request"<<std::endl;
        ac.cancelGoal();
        send_action = false;
        break;
      default:
        break;
    }

  }
  return 0;
}
