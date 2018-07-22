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

#ifndef AOTO_PILOT_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_H
#define AOTO_PILOT_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_H

//system include
#include <vector>
#include <mutex>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <messages/EnemyPos.h>

#include "common/main_interface.h"
#include "common/error_code.h"
#include "modules/perception/detection/armor_detection/armor_detection_base.h"

namespace rrts {
namespace perception {
namespace detection {

using rrts::common::ErrorCode;
using rrts::common::ErrorInfo;

/**
 * @brief This class achieved functions that can help to detect armors of RoboMaster vehicle.
 */
class ClusteringNode : public rrts::common::RRTS {
 public:
  explicit ClusteringNode(std::string name);
  /**
   * @brief Loading parameters from .prototxt file.
   */
  void LoadParam();
  /**
   * @brief Destructor
   */
  ErrorInfo DetectArmor(double &x, double &y, double &z, double &distance, double &pitch, double &yaw);
  void EnemyPoseCallBack(const messages::EnemyPosConstPtr &msg);
  void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  ~ClusteringNode() final;
 private:
  ErrorCode error_code_;
  ErrorInfo error_info_;
  std::mutex mutex_;
  double enemy_yaw_;
  messages::EnemyPos enemy_pos_;

  //ros
  ros::NodeHandle nh_;
  ros::NodeHandle nh_scan_;
  ros::Subscriber enemy_pos_sub_, scan_sub_;
  std::shared_ptr<tf::TransformListener> tf_listener_;
  ros::Publisher  enemy_pos_pub_;
};

} //namespace detection
} //namespace perception
} //namespace rrts

#endif // AOTO_PILOT_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_H
