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

#ifndef ROBORTS_DETECTION_ARMOR_DETECTION_NODE_H
#define ROBORTS_DETECTION_ARMOR_DETECTION_NODE_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include "actionlib/server/simple_action_server.h"
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/GimbalRate.h"
#include "roborts_msgs/ArmorDetectionAction.h"

#include "alg_factory/algorithm_factory.h"
#include "io/io.h"
#include "state/node_state.h"

#include "cv_toolbox.h"

#include "armor_detection_base.h"
#include "proto/armor_detection.pb.h"
#include "armor_detection_algorithms.h"
#include "gimbal_control.h"

namespace roborts_detection {

using roborts_common::NodeState;
using roborts_common::ErrorInfo;

class ArmorDetectionNode {
 public:
  explicit ArmorDetectionNode();
  /**
   * @brief Initializing armor detection algorithm.
   * @return Return the error information.
   */
  ErrorInfo Init();
  /**
   * @brief Actionlib server call back function.
   * @param data Command for control the algorithm thread.
   */
  void ActionCB(const roborts_msgs::ArmorDetectionGoal::ConstPtr &data);
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
  ~ArmorDetectionNode();
 protected:
 private:
  std::shared_ptr<ArmorDetectionBase> armor_detector_;
  std::thread armor_detection_thread_;
  unsigned int max_rotating_fps_;
  unsigned int min_rotating_detected_count_;
  unsigned int undetected_armor_delay_;

  //! state and error
  NodeState node_state_;
  ErrorInfo error_info_;
  bool initialized_;
  bool running_;
  std::mutex mutex_;
  std::condition_variable condition_var_;
  unsigned int undetected_count_;

  //! enemy information
  double x_;
  double y_;
  double z_;
  bool detected_enemy_;
  unsigned long demensions_;

  //ROS
  ros::NodeHandle nh_;
  ros::NodeHandle enemy_nh_;
  ros::Publisher enemy_info_pub_;
  std::shared_ptr<CVToolbox> cv_toolbox_;
  actionlib::SimpleActionServer<roborts_msgs::ArmorDetectionAction> as_;
  roborts_msgs::GimbalAngle gimbal_angle_;

  //! control model
  GimbalContrl gimbal_control_;
};
} //namespace roborts_detection

#endif //ROBORTS_DETECTION_ARMOR_DETECTION_NODE_H
