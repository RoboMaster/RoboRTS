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
#ifndef RRTS_SELF_TEST_H
#define RRTS_SELF_TEST_H


#include <thread>
#include <mutex>
#include <condition_variable>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/tf.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include "messages/SelfCheck.h"
#include "messages/CheckStatus.h"

#include "modules/driver/camera/camera_param.h"
#include "common/log.h"
#include "common/rrts.h"
#include "common/node_state.h"
#include "common/error_code.h"
#include "common/main_interface.h"

namespace rrts{
namespace driver {
namespace selfcheck {

using rrts::common::NodeState;
using rrts::common::ErrorInfo;

class SelfCheckNode : public rrts::common::RRTS {

 public:

  explicit SelfCheckNode(std::string name);
  ErrorInfo Init();
  bool LoadParams();
  bool SelfCheckCallback(messages::SelfCheck::Request  &req,
                         messages::SelfCheck::Response &res);
  void CheckLaser(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg);
  void CheckCamera(const sensor_msgs::ImageConstPtr &msg, unsigned int camera_id);
  void CheckStaticIMU(const nav_msgs::Odometry::ConstPtr &odom_msgs);
  ~SelfCheckNode() final;

 private:

  NodeState node_state_;
  ErrorInfo error_info_;

  //ros
  ros::CallbackQueue self_check_queue_;
  image_transport::ImageTransport *it_;
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber image_sub_;
  ros::Subscriber odom_sub_;
  std::vector<image_transport::Subscriber> subs_;
  ros::ServiceServer self_check_srv_;
  ros::ServiceClient check_status_srv_;

  ros::Duration check_duration_;
  ros::Time initial_check_time_;

  std::vector<bool> camera_status_vec_;
  bool camera_status_;

  bool laser_status_;

  nav_msgs::Odometry init_odom_;
  unsigned int odom_count_;
  bool imu_status_;



};
} //namespace selfcheck
} //namespace driver
} //namespace rrts

#endif //RRTS_SELF_TEST_H
