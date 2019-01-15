/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
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

#ifndef ROBORTS_DECISION_BEHAVIOR_TREE_H
#define ROBORTS_DECISION_BEHAVIOR_TREE_H

#include <chrono>

#include <ros/ros.h>

#include "behavior_node.h"

namespace roborts_decision{
/**
 * @brief Behavior tree class to initialize and execute the whole tree
 */
class BehaviorTree {
 public:
  /**
   * @brief Constructor of BehaviorTree
   * @param root_node root node of the behavior tree
   * @param cycle_duration tick duration of the behavior tree (unit ms)
   */
  BehaviorTree(BehaviorNode::Ptr root_node, int cycle_duration) :
      root_node_(root_node),
      cycle_duration_(cycle_duration) {}
  /**
   * @brief Loop to tick the behavior tree
   */
  void Run() {

    unsigned int frame = 0;
    while (ros::ok() ) {

      std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
      // Update the blackboard data
      ros::spinOnce();
      ROS_INFO("Frame : %d", frame);
      root_node_->Run();

      std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
      std::chrono::milliseconds execution_duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
      std::chrono::milliseconds sleep_time = cycle_duration_ - execution_duration;

      if (sleep_time > std::chrono::microseconds(0)) {

        std::this_thread::sleep_for(sleep_time);
        ROS_INFO("Excution Duration: %ld / %ld ms", cycle_duration_.count(), cycle_duration_.count());

      } else {

        ROS_WARN("The time tick once is %ld beyond the expected time %ld", execution_duration.count(), cycle_duration_.count());

      }
      ROS_INFO("----------------------------------");
      frame++;
    }
  }
 private:
  //! root node of the behavior tree
  BehaviorNode::Ptr root_node_;
  //! tick duration of the behavior tree (unit ms)
  std::chrono::milliseconds cycle_duration_;

};

}//namespace roborts_decision

#endif //ROBORTS_DECISION_BEHAVIOR_TREE_H
