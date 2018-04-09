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
#ifndef YOLO_H
#define YOLO_H
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <actionlib/server/simple_action_server.h>
#include <messages/InfantryInfoAction.h>
#include <tf/transform_listener.h>

#include "common/io.h"
#include "common/log.h"
#include "common/rrts.h"
#include "common/node_state.h"
#include "common/error_code.h"
#include "common/main_interface.h"

#include "modules/perception/detection/yolo/proto/yolo.pb.h"

namespace rrts{
namespace perception {
namespace detection {

using rrts::common::NodeState;
using rrts::common::ErrorInfo;

class YOLO : public rrts::common::RRTS {
  public:
  explicit YOLO(std::string name);
  ErrorInfo Init();
  void LoadParam();
  /**
   * @brief Actionlib server call back function.
   * @param data Command for control the algorithm thread.
   */
  void StartThread();
  /**
   * @brief Pausing the yolo thread when received command 2 in action_lib callback function.
   */
  void PauseThread();
  /**
   * @brief Stopping yolo thread.
   */
  void StopThread();
  /**
   * @brief Executing yolo.
   */
  void ActionCB(const messages::InfantryInfoGoal::ConstPtr &data);
  void ExecuteLoop();
  ~YOLO() final;
 private:
  //state and error
  NodeState node_state_;
  ErrorInfo error_info_;

  std::thread yolo_thread_;
  std::string datacfg_;
  std::string cfg_;
  std::string weights_;
  bool enable_debug_;
  bool initialized_;
  bool running_;
  bool updated_;
  int camera_id_;
  int object_num_;
  int *x_offset_;
  tf::Stamped<tf::Pose> object_pose_in_base_link_;
  std::condition_variable condition_var_;
  std::mutex mutex_;
  //ROS
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<messages::InfantryInfoAction> as_;
  std::shared_ptr<tf::TransformListener> tf_listener_;
};
} //namespace detection
} //namespace perception
} //namespace rrts

#endif //YOLO_H