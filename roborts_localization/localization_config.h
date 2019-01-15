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

#include <ros/ros.h>

#ifndef ROBORTS_LOCALIZATION_LOCALIZATION_CONFIG_H
#define ROBORTS_LOCALIZATION_LOCALIZATION_CONFIG_H

namespace roborts_localization {

// Get Parameters from ROS parameter server
struct LocalizationConfig {
  void GetParam(ros::NodeHandle *nh) {
      nh->param<std::string>("odom_frame", odom_frame_id, "odom");
      nh->param<std::string>("base_frame", base_frame_id, "base_link");
      nh->param<std::string>("global_frame", global_frame_id, "map");
      nh->param<std::string>("laser_topic_name", laser_topic_name, "scan");
      nh->param<std::string>("map_topic_name", map_topic_name, "map");
      nh->param<std::string>("init_pose_topic_name", init_pose_topic_name, "initialpose");
      nh->param<double>("transform_tolerance", transform_tolerance, 0.1);
      nh->param<double>("initial_pose_x", initial_pose_x, 1);
      nh->param<double>("initial_pose_y", initial_pose_y, 1);
      nh->param<double>("initial_pose_a", initial_pose_a, 0);
      nh->param<double>("initial_cov_xx", initial_cov_xx, 0.1);
      nh->param<double>("initial_cov_yy", initial_cov_yy, 0.1);
      nh->param<double>("initial_cov_aa", initial_cov_aa, 0.1);
      nh->param<bool>("enable_uwb", enable_uwb, false);
      nh->param<std::string>("uwb_frame_id", uwb_frame_id, "uwb");
      nh->param<std::string>("uwb_topic_name", uwb_topic_name, "uwb");
      nh->param<bool>("use_sim_uwb", use_sim_uwb, false);
      nh->param<int>("uwb_correction_frequency", uwb_correction_frequency, 20);
      nh->param<bool>("publish_visualize", publish_visualize, true);
  }
  std::string odom_frame_id;
  std::string base_frame_id;
  std::string global_frame_id;

  std::string laser_topic_name;
  std::string map_topic_name;
  std::string init_pose_topic_name;

  double transform_tolerance;

  double initial_pose_x;
  double initial_pose_y;
  double initial_pose_a;
  double initial_cov_xx;
  double initial_cov_yy;
  double initial_cov_aa;

  bool publish_visualize;

  bool enable_uwb;
  std::string uwb_frame_id;
  std::string uwb_topic_name;
  bool use_sim_uwb;
  int uwb_correction_frequency;

};

}// roborts_localization

#endif //ROBORTS_LOCALIZATION_LOCALIZATION_CONFIG_H
