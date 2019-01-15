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

#ifndef ROBORTS_LOCALIZATION_LOCALIZATION_NODE_H
#define ROBORTS_LOCALIZATION_LOCALIZATION_NODE_H

#include <memory>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GetMap.h>

#include "log.h"
#include "localization_config.h"
#include "amcl/amcl.h"
#include "localization_math.h"
#include "types.h"

#define THREAD_NUM 4 // ROS SPIN THREAD NUM

namespace roborts_localization {

class LocalizationNode {
 public:

  /**
   * @brief Localization Node construct function
   * @param name Node name
   */
  LocalizationNode(std::string name);

  /**
   * @brief Localization initialization
   * @return Returns true if initialize success
   */
  bool Init();

  /**
   * @brief Laser scan messages callback function (as main loop in localization node)
   * @param laser_scan_msg Laser scan data
   */
  void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg);

  /**
   * @brief Manually initialize localization init_pose
   * @param init_pose_msg Init pose (2D Pose Estimate button in RViz)
   */
  void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &init_pose_msg);

  /**
   * @brief Publish visualize messages
   */
  void PublishVisualize();

  /**
   * @brief Publish transform information between odom_frame and global_frame
   * @return True if no errors
   */
  bool PublishTf();

 private:

  bool GetPoseFromTf(const std::string &target_frame,
                     const std::string &source_frame,
                     const ros::Time &timestamp,
                     Vec3d &pose);

  bool GetStaticMap();

  bool GetLaserPose();

  void TransformLaserscanToBaseFrame(double &angle_min,
                                     double &angle_increment,
                                     const sensor_msgs::LaserScan &laser_scan_msg);
 private:
  //Mutex
  std::mutex mutex_;

  //Algorithm object
  std::unique_ptr<Amcl> amcl_ptr_;

  //ROS Node handle
  ros::NodeHandle nh_;

  //TF
  std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_ptr_;
  std::unique_ptr<tf::TransformListener> tf_listener_ptr_;

  //ROS Subscriber
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber uwb_pose_sub_;
  ros::Subscriber ground_truth_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laser_scan_sub_;
  std::unique_ptr<tf::MessageFilter<sensor_msgs::LaserScan>> laser_scan_filter_;

  //ROS Publisher
  ros::Publisher pose_pub_;
  ros::Publisher particlecloud_pub_;
  ros::Publisher distance_map_pub_;
  ros::Publisher beam_sample_pub_;

  //ROS Service
  ros::ServiceServer random_heading_srv_;
  ros::ServiceClient static_map_srv_;

  //Parameters
  std::string odom_frame_;
  std::string global_frame_;
  std::string base_frame_;
  std::string laser_topic_;
  Vec3d init_pose_;
  Vec3d init_cov_;
//  bool enable_uwb_;
  ros::Duration transform_tolerance_;
  bool publish_visualize_;

  //Status
  bool initialized_ = false;
  bool map_init_ = false;
  bool laser_init_ = false;
  bool first_map_received_ = false;
  bool latest_tf_valid_ = false;
  bool sent_first_transform_ = false;
  bool publish_first_distance_map_ = false;

  //Data
  HypPose hyp_pose_;
  geometry_msgs::PoseArray particlecloud_msg_;
  geometry_msgs::PoseStamped pose_msg_;
  ros::Time last_laser_msg_timestamp_;
  tf::Transform latest_tf_;
  tf::Stamped<tf::Pose> latest_odom_pose_;
};

}// roborts_localization

#endif // ROBORTS_LOCALIZATION_LOCALIZATION_NODE_H
