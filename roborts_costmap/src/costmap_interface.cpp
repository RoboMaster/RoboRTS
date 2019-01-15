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
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include "costmap_parameter_setting.pb.h"
#include "costmap_interface.h"

namespace roborts_costmap {

CostmapInterface::CostmapInterface(std::string map_name,
                                   tf::TransformListener &tf,
                                   std::string config_file) :
    layered_costmap_(nullptr),
    name_(map_name),
    tf_(tf),
    config_file_(config_file),
    stop_updates_(false),
    initialized_(true),
    stopped_(false),
    robot_stopped_(false),
    map_update_thread_(NULL),
    last_publish_(0),
    dist_behind_robot_threshold_to_care_obstacles_(0.05),
    is_debug_(false),
    map_update_thread_shutdown_(false) {
  std::string tf_error;
  ros::NodeHandle private_nh(map_name);
  LoadParameter();
  layered_costmap_ = new CostmapLayers(global_frame_, is_rolling_window_, is_track_unknown_);
  layered_costmap_->SetFilePath(config_file_inflation_);
  ros::Time last_error = ros::Time::now();
  while (ros::ok() && !tf_.waitForTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), \
         ros::Duration(0.01), &tf_error)) {
    ros::spinOnce();
    if (last_error + ros::Duration(5.0) < ros::Time::now()) {
      last_error = ros::Time::now();
    }
    tf_error.clear();
  }
  if (has_static_layer_) {
    Layer *plugin_static_layer;
    plugin_static_layer = new StaticLayer;
    layered_costmap_->AddPlugin(plugin_static_layer);
    plugin_static_layer->Initialize(layered_costmap_, map_name + "/" + "static_layer", &tf_);
  }
  if (has_obstacle_layer_) {
    Layer *plugin_obstacle_layer = new ObstacleLayer;
    layered_costmap_->AddPlugin(plugin_obstacle_layer);
    plugin_obstacle_layer->Initialize(layered_costmap_, map_name + "/" + "obstacle_layer", &tf_);
  }
  Layer *plugin_inflation_layer = new InflationLayer;
  layered_costmap_->AddPlugin(plugin_inflation_layer);
  plugin_inflation_layer->Initialize(layered_costmap_, map_name + "/" + "inflation_layer", &tf_);
  SetUnpaddedRobotFootprint(footprint_points_);
  stop_updates_ = false;
  initialized_ = true;
  stopped_ = false;
  robot_stopped_ = false;
  map_update_thread_shutdown_ = false;
  timer_ = private_nh.createTimer(ros::Duration(0.1), &CostmapInterface::DetectMovement, this);
  if (is_debug_) {
    // special values:
    cost_translation_table_[0] = 0;  // NO obstacle
    cost_translation_table_[253] = 99;  // INSCRIBED obstacle
    cost_translation_table_[254] = 100;  // LETHAL obstacle
    cost_translation_table_[255] = -1;  // UNKNOWN

    // regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    for (int i = 1; i < 253; i++) {
      cost_translation_table_[i] = char(1 + (97 * (i - 1)) / 251);
    }
  }
  costmap_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>(name_ + "/costmap", 10);
  map_update_thread_ = new std::thread(std::bind(&CostmapInterface::MapUpdateLoop, this, map_update_frequency_));
  if (is_rolling_window_) {
    layered_costmap_->ResizeMap((unsigned int) map_width_ / map_resolution_,
                                (unsigned int) map_height_ / map_resolution_,
                                map_resolution_,
                                map_origin_x_,
                                map_origin_y_);
  }
}

void CostmapInterface::LoadParameter() {

  ParaCollection ParaCollectionConfig;
  roborts_common::ReadProtoFromTextFile(config_file_.c_str(), \
                                &ParaCollectionConfig);


  config_file_ = ros::package::getPath("roborts_costmap") + \
      ParaCollectionConfig.para_costmap_interface().inflation_file_path();

  map_update_frequency_ = ParaCollectionConfig.para_costmap_interface().map_update_frequency();
  global_frame_ = ParaCollectionConfig.para_costmap_interface().global_frame();
  robot_base_frame_ = ParaCollectionConfig.para_costmap_interface().robot_base_frame();
  footprint_padding_ = ParaCollectionConfig.para_costmap_interface().footprint_padding();
  transform_tolerance_ = ParaCollectionConfig.para_costmap_interface().transform_tolerance();
  is_rolling_window_ = ParaCollectionConfig.para_costmap_interface().is_rolling_window();
  is_debug_ = ParaCollectionConfig.para_basic().is_debug();
  is_track_unknown_ = ParaCollectionConfig.para_costmap_interface().is_tracking_unknown();
  has_obstacle_layer_ = ParaCollectionConfig.para_costmap_interface().has_obstacle_layer();
  has_static_layer_ = ParaCollectionConfig.para_costmap_interface().has_static_layer();
  map_width_ = ParaCollectionConfig.para_costmap_interface().map_width();
  map_height_ = ParaCollectionConfig.para_costmap_interface().map_height();
  map_origin_x_ = ParaCollectionConfig.para_costmap_interface().map_origin_x();
  map_origin_y_ = ParaCollectionConfig.para_costmap_interface().map_origin_y();
  map_resolution_ = ParaCollectionConfig.para_costmap_interface().map_resolution();


  config_file_inflation_ = ros::package::getPath("roborts_costmap") + \
      ParaCollectionConfig.para_costmap_interface().inflation_file_path();

  geometry_msgs::Point point;
  for (auto i = 0; i < ParaCollectionConfig.footprint().point().size(); i++) {
    point.x = ParaCollectionConfig.footprint().point(i).x();
    point.y = ParaCollectionConfig.footprint().point(i).y();
    point.z = 0.0;
    footprint_points_.push_back(point);
  }
}

void CostmapInterface::SetUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon &footprint) {
  SetUnpaddedRobotFootprint(ToPointVector(footprint));
}

CostmapInterface::~CostmapInterface() {
  timer_.stop();
  map_update_thread_shutdown_ = true;
  if (map_update_thread_ != NULL) {
    map_update_thread_->join();
    delete map_update_thread_;
  }
  delete layered_costmap_;
}

void CostmapInterface::SetUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point> &points) {
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  PadFootprint(padded_footprint_, footprint_padding_);
  layered_costmap_->SetFootprint(padded_footprint_);
}

void CostmapInterface::DetectMovement(const ros::TimerEvent &event) {
  tf::Stamped<tf::Pose> new_pose;
  if (!GetRobotPose(new_pose)) {
    robot_stopped_ = false;
  } else if (fabs((old_pose_.getOrigin() - new_pose.getOrigin()).length()) < 1e-3 \
 && fabs(old_pose_.getRotation().angle(new_pose.getRotation())) < 1e-3) {
    old_pose_ = new_pose;
    robot_stopped_ = true;
  } else {
    old_pose_ = new_pose;
    robot_stopped_ = false;
  }
}

void CostmapInterface::MapUpdateLoop(double frequency) {
  if (frequency <= 0.0) {
    ROS_ERROR("Frequency must be positive in MapUpdateLoop.");
  }
  ros::NodeHandle nh;
  ros::Rate r(frequency);
  while (nh.ok() && !map_update_thread_shutdown_) {
    struct timeval start, end;
    gettimeofday(&start, NULL);
    UpdateMap();
    gettimeofday(&end, NULL);
    r.sleep();
    if (r.cycleTime() > ros::Duration(1 / frequency)) {
      if (is_debug_) {
        ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds",\
        frequency, r.cycleTime().toSec());
      }
    }
    double x, y;
    Costmap2D *temp_costmap = layered_costmap_->GetCostMap();
    unsigned char *data = temp_costmap->GetCharMap();
    temp_costmap->Map2World(0, 0, x, y);
    grid_.header.frame_id = global_frame_;
    grid_.header.stamp = ros::Time::now();
    if (is_rolling_window_) {
      grid_.info.resolution = map_resolution_;
      grid_.info.width = map_width_ / map_resolution_;
      grid_.info.height = map_height_ / map_resolution_;
      grid_.info.origin.position.x = x - map_resolution_ * 0.5;
      grid_.info.origin.position.y = y - map_resolution_ * 0.5;
      grid_.info.origin.position.z = 0;
      grid_.info.origin.orientation.w = 1.0;
      grid_.data.resize(grid_.info.width * grid_.info.height);
    } else {
      auto resolution = temp_costmap->GetResolution();
      auto map_width = temp_costmap->GetSizeXCell();
      auto map_height = temp_costmap->GetSizeYCell();
      grid_.info.resolution = resolution;
      grid_.info.width = map_width;
      grid_.info.height = map_height;
      grid_.info.origin.position.x = temp_costmap->GetOriginX();
      grid_.info.origin.position.y = temp_costmap->GetOriginY();
      grid_.info.origin.position.z = 0;
      grid_.info.origin.orientation.w = 1.0;
      grid_.data.resize(map_width * map_height);
    }
    for (size_t i = 0; i < grid_.data.size(); i++) {
      grid_.data[i] = cost_translation_table_[data[i]];
    }
    costmap_pub_.publish(grid_);
  }
}

void CostmapInterface::UpdateMap() {
  if (!stop_updates_) {
    tf::Stamped<tf::Pose> pose;
    if (GetRobotPose(pose)) {
      double x = pose.getOrigin().x(), y = pose.getOrigin().y(), \
 yaw = tf::getYaw(pose.getRotation());
      layered_costmap_->UpdateMap(x, y, yaw);
      geometry_msgs::PolygonStamped footprint;
      footprint.header.frame_id = global_frame_;
      footprint.header.stamp = ros::Time::now();
      TransformFootprint(x, y, yaw, padded_footprint_, footprint);
      initialized_ = true;
    }
  }
}

void CostmapInterface::Start() {
  auto *plugins = layered_costmap_->GetPlugins();
  if (stopped_) {
    for (auto plugin = plugins->begin(); plugin != plugins->end(); ++plugin) {
      (*plugin)->Activate();
    }
    stopped_ = false;
  }
  stop_updates_ = false;
  ros::Rate r(100.0);
  while (ros::ok() && !initialized_) {
    r.sleep();
  }
}

void CostmapInterface::Stop() {
  stop_updates_ = true;
  auto *plugins = layered_costmap_->GetPlugins();
  for (auto plugin = plugins->begin(); plugin != plugins->end(); ++plugin) {
    (*plugin)->Deactivate();
  }
  initialized_ = false;
  stopped_ = true;
}

void CostmapInterface::Pause() {
  stop_updates_ = true;
  initialized_ = false;
}

void CostmapInterface::Resume() {
  stop_updates_ = false;
  ros::Rate r(100.0);
  while (!initialized_) {
    r.sleep();
  }
}

void CostmapInterface::ResetLayers() {
  Costmap2D *master = layered_costmap_->GetCostMap();
  master->ResetPartMap(0, 0, master->GetSizeXCell(), master->GetSizeYCell());
  auto plugins = layered_costmap_->GetPlugins();
  for (auto plugin = (*plugins).begin(); plugin != (*plugins).end(); ++plugin) {
    (*plugin)->Reset();
  }
}

bool CostmapInterface::GetRobotPose(tf::Stamped<tf::Pose> &global_pose) const {
  global_pose.setIdentity();
  tf::Stamped<tf::Pose> robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = robot_base_frame_;
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time = ros::Time::now();
  try {
    tf_.transformPose(global_frame_, robot_pose, global_pose);
  }
  catch (tf::LookupException &ex) {
    ROS_ERROR("No Transform Error looking up robot pose: %s", ex.what());
    return false;
  }
  catch (tf::ConnectivityException &ex) {
    ROS_ERROR("Connectivity Error looking up robot pose: %s", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException &ex) {
    ROS_ERROR("Extrapolation Error looking up robot pose: %s", ex.what());
    return false;
  }
  // check global_pose timeout
  //if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_) {
  //  LOG_WARNING << "Interface transform timeout. Current time: " << current_time.toSec() << ", global_pose stamp: "
  //              << global_pose.stamp_.toSec() << ", tolerance: " << transform_tolerance_;
  //  return false;
  //}
  return true;
}

bool CostmapInterface::GetRobotPose(geometry_msgs::PoseStamped &global_pose) const {
  tf::Stamped<tf::Pose> tf_global_pose;
  if (GetRobotPose(tf_global_pose)) {
    tf::poseStampedTFToMsg(tf_global_pose, global_pose);
    return true;
  } else {
    return false;
  }
}

void CostmapInterface::GetOrientedFootprint(std::vector<geometry_msgs::Point> &oriented_footprint) const {
  tf::Stamped<tf::Pose> global_pose;
  if (!GetRobotPose(global_pose)) {
    return;
  }
  double yaw = tf::getYaw(global_pose.getRotation());
  TransformFootprint(global_pose.getOrigin().x(), global_pose.getOrigin().y(), yaw,
                     padded_footprint_, oriented_footprint);
}

void CostmapInterface::GetFootprint(std::vector<Eigen::Vector3f> &footprint) {
  std::vector<geometry_msgs::Point> ros_footprint = GetRobotFootprint();
  Eigen::Vector3f point;
  for (auto it: ros_footprint) {
    point << it.x, it.y, it.z;
    footprint.push_back(point);
  }
}

void CostmapInterface::GetOrientedFootprint(std::vector<Eigen::Vector3f> &footprint) {
  std::vector<geometry_msgs::Point> oriented_footprint;
  GetOrientedFootprint(oriented_footprint);
  Eigen::Vector3f position;
  for (auto it: oriented_footprint) {
    position << it.x, it.y, it.z;
    footprint.push_back(position);
  }
}

bool CostmapInterface::GetRobotPose(RobotPose &pose) {
  tf::Stamped<tf::Pose> global_pose;
  if (GetRobotPose(global_pose)) {
    pose.time = global_pose.stamp_;
    pose.frame_id = global_pose.frame_id_;
    pose.position << global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), global_pose.getOrigin().getZ();
    auto mat = global_pose.getBasis();
    pose.rotation << mat.getRow(0).getX(), mat.getRow(0).getY(), mat.getRow(0).getZ(), \
        mat.getRow(1).getX(), mat.getRow(1).getY(), mat.getRow(1).getZ(), \
        mat.getRow(2).getX(), mat.getRow(2).getY(), mat.getRow(2).getZ();
    return true;
  }
  return false;
}

geometry_msgs::PoseStamped CostmapInterface::Pose2GlobalFrame(const geometry_msgs::PoseStamped &pose_msg) {
  tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
  poseStampedMsgToTF(pose_msg, tf_pose);

  tf_pose.stamp_ = ros::Time();
  try {
    tf_.transformPose(global_frame_, tf_pose, global_tf_pose);
  }
  catch (tf::TransformException &ex) {
    return pose_msg;
  }
  geometry_msgs::PoseStamped global_pose_msg;
  tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);
  return global_pose_msg;
}

void CostmapInterface::ClearCostMap() {
  std::vector<Layer *> *plugins = layered_costmap_->GetPlugins();
  tf::Stamped<tf::Pose> pose;
  if (GetRobotPose(pose) == false) {
    return;
  }
  double pose_x = pose.getOrigin().x();
  double pose_y = pose.getOrigin().y();

  for (std::vector<roborts_costmap::Layer *>::iterator plugin_iter = plugins->begin();
       plugin_iter != plugins->end();
       ++plugin_iter) {
    roborts_costmap::Layer *plugin = *plugin_iter;
    if (plugin->GetName().find("obstacle") != std::string::npos) {
      CostmapLayer *costmap_layer_ptr = (CostmapLayer *) plugin;
      ClearLayer(costmap_layer_ptr, pose_x, pose_y);
    }

  }
}

void CostmapInterface::ClearLayer(CostmapLayer *costmap_layer_ptr, double pose_x, double pose_y) {
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_layer_ptr->GetMutex()));
  double reset_distance = 0.1;
  double start_point_x = pose_x - reset_distance / 2;
  double start_point_y = pose_y - reset_distance / 2;
  double end_point_x = start_point_x + reset_distance;
  double end_point_y = start_point_y + reset_distance;

  int start_x, start_y, end_x, end_y;
  costmap_layer_ptr->World2MapNoBoundary(start_point_x, start_point_y, start_x, start_y);
  costmap_layer_ptr->World2MapNoBoundary(end_point_x, end_point_y, end_x, end_y);

  unsigned char *grid = costmap_layer_ptr->GetCharMap();
  for (int x = 0; x < (int) costmap_layer_ptr->GetSizeXCell(); x++) {
    bool xrange = x > start_x && x < end_x;

    for (int y = 0; y < (int) costmap_layer_ptr->GetSizeYCell(); y++) {
      if (xrange && y > start_y && y < end_y)
        continue;
      int index = costmap_layer_ptr->GetIndex(x, y);
      if (grid[index] != NO_INFORMATION) {
        grid[index] = NO_INFORMATION;
      }
    }
  }

  double ox = costmap_layer_ptr->GetOriginX(), oy = costmap_layer_ptr->GetOriginY();
  double width = costmap_layer_ptr->GetSizeXWorld(), height = costmap_layer_ptr->GetSizeYWorld();
  costmap_layer_ptr->AddExtraBounds(ox, oy, ox + width, oy + height);
}
} //namespace roborts_costmap
