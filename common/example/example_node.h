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
#ifndef COMMON_MAIN_TEST_H
#define COMMON_MAIN_TEST_H

#include <thread>
#include <mutex>

#include "boost/thread.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib/server/simple_action_server.h"


#include "messages/exampleAction.h"
#include "common/rrts.h"
#include "common/log.h"
#include "common/error_code.h"

#include "common/example/example_base.h"
#include "common/example/example_algorithm.h"
#include "common/algorithm_factory.h"
#include "common/node_state.h"

namespace rrts{
namespace common {

class MainTest : public rrts::common::RRTS {
 public:
  MainTest( std::string name);
  ~MainTest();

 private:

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  actionlib::SimpleActionServer<messages::exampleAction> as_;

  void CB(const geometry_msgs::PoseStamped::ConstPtr & pose);
  void ActionCB(const messages::exampleGoal::ConstPtr &command);

  void StartThread();
  void StopThread();
  void FunctionThread();

  void SetNodeState(const NodeState& node_state);
  NodeState GetNodeState();

  void SetReturnState(const ErrorInfo return_state);
  ErrorInfo GetReturnState();

  std::thread thread_;
  std::unique_ptr<ExampleBase> selected_algorithm_ptr_;

  NodeState node_state_;
  ErrorInfo return_state_;
  std::mutex node_state_mtx_;
  std::mutex return_state_mtx_;

  bool running_;
  bool initialized_;

  std::chrono::microseconds cycle_duration_;


};
}
}

#endif //COMMON_MAIN_TEST_H
