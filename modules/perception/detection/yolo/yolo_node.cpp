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
#include <string>

#include "c2cpp.h"
#include "yolo_node.hpp"
namespace rrts{
namespace perception {
namespace detection {

YOLO::YOLO(std::string name): rrts::common::RRTS::RRTS(name),
                              node_state_(rrts::common::IDLE),
                              running_(false),
                              initialized_(false),
                              updated_(false),
                              object_num_(0),
                              x_offset_(NULL),
as_(nh_, name+"_action", boost::bind(&YOLO::ActionCB, this, _1), false){
  tf_listener_ = std::make_shared<tf::TransformListener>(ros::Duration(1));
  if (Init().IsOK()) {
    initialized_ = true;
    node_state_ = rrts::common::IDLE;
  } else {
    LOG_ERROR << "armor_detection_node initalized failed!";
    node_state_ = rrts::common::FAILURE;
  }
  NOTICE("Waiting for input command in armor_detection_client...")
  as_.start();
}
ErrorInfo YOLO::Init() {
  LoadParam();
  InitYOLO(datacfg_.c_str(), cfg_.c_str(), weights_.c_str(), .5, enable_debug_);
  return ErrorInfo(rrts::common::ErrorCode::OK);
}
void YOLO::LoadParam() {
  //read parameters
  YOLOConfig yolo_config_;
  std::string file_name =
      "/modules/perception/detection/yolo/config/yolo.prototxt";
  bool read_state = rrts::common::ReadProtoFromTextFile(file_name, &yolo_config_);
  CHECK(read_state) << "Cannot open " << file_name;
  datacfg_      = yolo_config_.datacfg();
  cfg_          = yolo_config_.cfg();
  weights_      = yolo_config_.weights();
  enable_debug_ = yolo_config_.enable_debug();
  camera_id_    = yolo_config_.init_camera_id();
}
void YOLO::ActionCB(const messages::InfantryInfoGoal::ConstPtr &data) {
  messages::InfantryInfoFeedback feedback;
  messages::InfantryInfoResult result;

  if(!initialized_){
    as_.publishFeedback(feedback);
    LOG_INFO<<"Initialization Failed, Failed to execute action!";
    return;
  }

  switch (data->command) {
    case 1:
      StartThread();
      break;
    case 2:
      PauseThread();
      break;
    case 3:
      StopThread();
      break;
    default:
      break;
  }

  while(ros::ok()) {
    if(as_.isPreemptRequested()) {
      as_.setPreempted();
      return;
    }

    if (updated_) {
      std::lock_guard<std::mutex> guard(mutex_);
      feedback.object_pos.pose.orientation.x = object_pose_in_base_link_.getRotation().x();
      feedback.object_pos.pose.orientation.y = object_pose_in_base_link_.getRotation().y();
      feedback.object_pos.pose.orientation.z = object_pose_in_base_link_.getRotation().z();
      feedback.object_pos.pose.orientation.w = object_pose_in_base_link_.getRotation().w();
      feedback.object_pos.pose.position.x = 0;
      feedback.object_pos.pose.position.y = 0;
      feedback.object_pos.pose.position.z = 0;
      std::string frame_name = "camera" + std::to_string(camera_id_);
      feedback.object_pos.header.frame_id = frame_name;
      feedback.object_pos.header.stamp = ros::Time::now();
      as_.publishFeedback(feedback);
      updated_ = false;
    }
  }
}
void YOLO::ExecuteLoop() {
  while(running_) {
    if (node_state_ == NodeState::RUNNING) {
      RunYOLO(enable_debug_, camera_id_, &x_offset_, &object_num_);
      for (unsigned int i = 0;i < object_num_; i++) {
        if (x_offset_[i] != 0) {
          std::lock_guard<std::mutex> guard(mutex_);
          tf::Stamped<tf::Pose> pose_in;
          tf::Quaternion q;
          std::string frame_name = "camera" + std::to_string(camera_id_);
          pose_in.frame_id_ = frame_name;
          double yaw = ((x_offset_[i] - 640) / 1280 * 22.5) * M_PI / 180;
          q.setRPY(0, 0, yaw);
          pose_in.setRotation(q);
          pose_in.stamp_ = ros::Time(0);
          try{
            tf_listener_->transformPose("base_link", pose_in, object_pose_in_base_link_);
          }
          catch(tf::TransformException& ex) {
            ROS_WARN("Transformation failed!");
          }
          updated_ = true;
        }
      }
      if(x_offset_){
        free(x_offset_);
        x_offset_ = NULL;
      }
      object_num_ = 0;
    } else if (node_state_ == NodeState::PAUSE) {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_var_.wait(lock);
    }
  }
}
void YOLO::StartThread() {
  LOG_INFO << "YOLO thread started!";
  running_ = true;
  if(node_state_ == NodeState::IDLE)
    yolo_thread_ = std::thread(&YOLO::ExecuteLoop, this);
  node_state_ = NodeState::RUNNING;
  condition_var_.notify_one();
}
void YOLO::PauseThread() {
  LOG_INFO << "YOLO thread paused!";
  node_state_ = NodeState::PAUSE;
}
void YOLO::StopThread() {
  node_state_ = NodeState::IDLE;
  running_ = false;
  if (yolo_thread_.joinable()) {
    yolo_thread_.join();
  }
}
YOLO::~YOLO() {
  StopThread();
}
} //namespace detection
} //namespace perception
} //namespace rrts

MAIN(rrts::perception::detection::YOLO, "yolo_node")