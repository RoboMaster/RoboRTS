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

#ifndef MODULES_PERCEPTION_DETECTION_ARMORDETECTION_NODE_H
#define MODULES_PERCEPTION_DETECTION_ARMORDETECTION_NODE_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include "actionlib/server/simple_action_server.h"
#include "messages/EnemyPos.h"
#include "messages/ArmorDetectionAction.h"

#include "modules/perception/detection/armor_detection/armor_detection_base.h"
#include "modules/perception/detection/armor_detection/proto/armor_detection.pb.h"
#include "modules/perception/detection/armor_detection/armor_detection_algorithms.h"
#include "modules/perception/detection/util/cv_toolbox.h"

#include "common/algorithm_factory.h"
#include "common/io.h"
#include "common/log.h"
#include "common/rrts.h"
#include "common/node_state.h"
#include "common/main_interface.h"

namespace rrts{
namespace perception {
namespace detection {

using rrts::common::NodeState;
using rrts::common::ErrorInfo;

class ArmorDetectionNode : public rrts::common::RRTS {
 public:
  explicit ArmorDetectionNode(std::string name);
  /**
   * @brief Initializing armor detection algorithm.
   * @return Return the error information.
   */
  ErrorInfo Init();
  /**
   * @brief Actionlib server call back function.
   * @param data Command for control the algorithm thread.
   */
  void ActionCB(const messages::ArmorDetectionGoal::ConstPtr &data);
  /**
   * @brief Starting the armor detection thread.
   */
  void StartThread();
  /**
   * @brief Pausing the armor detection thread when received command 2 in action_lib callback function.
   */
  void PauseThread();
  /**
   * @brief Stopping armor detection thread.
   */
  void StopThread();
  /**
   * @brief Executing the armor detection algorithm.
   */
  void ExecuteLoop();
  /**
   * @brief Publishing enemy pose information that been calculated by the armor detection algorithm.
   */
  void PublishMsgs();
  ~ArmorDetectionNode() final;
 protected:
 private:
  std::shared_ptr<ArmorDetectionBase> armor_detector_;
  std::thread armor_detection_thread_;

  //state and error
  NodeState node_state_;
  ErrorInfo error_info_;
  bool initialized_;
  bool running_;
  std::mutex mutex_;
  std::condition_variable condition_var_;

  //enemy information
  double distance_;
  double pitch_;
  double yaw_;
  bool detected_enemy_;
  unsigned long demensions_;

  //ROS
  ros::NodeHandle nh_;
  ros::Publisher enemy_info_pub_;
  actionlib::SimpleActionServer<messages::ArmorDetectionAction> as_;
  messages::EnemyPos enemy_pos;
};
} //namespace detection
} //namespace perception
} //namespace rrts

#endif //MODULES_PERCEPTION_DETECTION_ARMORDETECTION_NODE_H
