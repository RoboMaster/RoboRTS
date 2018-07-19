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

#ifndef MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_TREE_H
#define MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_TREE_H

#include <chrono>

#include <ros/ros.h>

#include "modules/decision/behavior_tree/behavior_node.h"

namespace rrts{
namespace decision {
class BehaviorTree {
 public:
  BehaviorTree(BehaviorNode::Ptr root_node, int cycle_duration) :
      root_node_(root_node),
      cycle_duration_(cycle_duration),
      running_(false) {}
  void Execute() {
    running_ = true;
    unsigned int frame = 0;
    while (ros::ok() && running_) {

      std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
      ros::spinOnce();
      LOG_INFO << "Frame " << frame << ":";
      root_node_->Run();

      std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
      std::chrono::microseconds execution_duration =
          std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
      std::chrono::microseconds sleep_time = cycle_duration_ - execution_duration;

      if (sleep_time > std::chrono::microseconds(0)) {
        std::this_thread::sleep_for(sleep_time);
        LOG_INFO << "sleep: " << sleep_time.count() << "us";
      } else {
        LOG_WARNING << "The time planning once is " << execution_duration.count() << " beyond the expected time "
                  << cycle_duration_.count();
      }

      LOG_INFO << "----------------------------------";
      frame++;
    }
  }
 private:
  BehaviorNode::Ptr root_node_;
  std::chrono::milliseconds cycle_duration_;
  bool running_;
};

}//namespace decision
}//namespace rrts

#endif //MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_TREE_H
