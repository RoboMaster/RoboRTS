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

#ifndef MODULES_PERCEPTION_DETECTION_COLORDETECTION_NODE_H
#define MODULES_PERCEPTION_DETECTION_COLORDETECTION_NODE_H

#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <actionlib/server/simple_action_server.h>
#include <messages/EnemyDirectionAction.h>
#include <tf/transform_listener.h>

#include "common/io.h"
#include "common/log.h"
#include "common/rrts.h"
#include "common/node_state.h"
#include "common/error_code.h"
#include "common/main_interface.h"

#include "modules/perception/detection/util/cv_toolbox.h"
#include "modules/perception/detection/color_detection/proto/color_detection.pb.h"

namespace rrts{
namespace perception {
namespace detection {

using rrts::common::NodeState;
using rrts::common::ErrorInfo;

class ColorDetectionNode : public rrts::common::RRTS {
 public:
  explicit ColorDetectionNode(std::string name);
  ErrorInfo Init();
  void ActionCB(const messages::EnemyDirectionGoal::ConstPtr &data);
  void StartThread();
  void PauseThread();
  void StopThread();
  void ExecuteLoop();
  int PixelSum(cv::Mat src);
  ~ColorDetectionNode() final;
 protected:
 private:
  cv::Mat src_;
  CVToolbox cv_toolbox_;
  std::thread armor_detection_thread_;

  //state and error
  NodeState node_state_;
  ErrorInfo error_info_;

  bool initialized_;
  bool running_;
  bool detected_enemy_;
  std::mutex mutex_;
  std::condition_variable condition_var_;
  int camera_id_;
  unsigned int enemy_color_;
  bool enable_debug_;
  bool using_hsv_;
  unsigned int threshold_;
  unsigned int min_pixel_number_;

  //enemy information
  short int enemy_direction_;

  //ROS
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<messages::EnemyDirectionAction> as_;
};
} //namespace detection
} //namespace perception
} //namespace rrts

#endif //MODULES_PERCEPTION_DETECTION_COLORDETECTION_NODE_H
