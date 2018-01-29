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

#include "modules/perception/detection/armor_detection/armor_detection_node.h"

namespace rrts{
namespace perception {
namespace detection {

ArmorDetectionNode::ArmorDetectionNode(std::string name):
    node_state_(rrts::common::IDLE),
    demensions_(3),
    initialized_(false),
    rrts::common::RRTS::RRTS(name),
    as_(nh_, name+"_action", boost::bind(&ArmorDetectionNode::ActionCB, this, _1), false) {
  initialized_ = false;
  translation_.resize(demensions_, 0);
  rotation_.resize(demensions_, 0);
  if (Init().IsOK()) {
    initialized_ = true;
    node_state_ = rrts::common::IDLE;
  } else {
    LOG_ERROR << "armor_detection_node initalized failed!";
    node_state_ = rrts::common::FAILURE;
  }
  as_.start();
}

ErrorInfo ArmorDetectionNode::Init() {
  enemy_info_pub_ = nh_.advertise<messages::EnemyPos>("enemy_pos", 100);

  ArmorDetectionAlgorithms armor_detection_algorithms;
  std::string file_name = "modules/perception/detection/armor_detection/config/armor_detection.prototxt";
  bool read_state = rrts::common::ReadProtoFromTextFile(file_name, &armor_detection_algorithms);
  if (!read_state) {
    LOG_ERROR << "Cannot open " << file_name;
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  }

  //create the selected algorithms
  std::string selected_algorithm = armor_detection_algorithms.selected_algorithm();
  armor_detector_ = rrts::common::AlgorithmFactory<ArmorDetectionBase>::CreateAlgorithm(selected_algorithm);

  if (armor_detector_ == nullptr) {
    LOG_ERROR << "Create armor_detector_ pointer failed!";
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  } else
    return ErrorInfo(ErrorCode::OK);
}

void ArmorDetectionNode::ActionCB(const messages::ArmorDetectionGoal::ConstPtr &data) {
  messages::ArmorDetectionFeedback feedback;
  messages::ArmorDetectionResult result;

  if(!initialized_){
    feedback.error_code = error_info_.error_code();
    feedback.error_msg  = error_info_.error_msg();
    as_.publishFeedback(feedback);
    as_.setAborted(result, feedback.error_msg);
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

    if (!translation_.empty() && !rotation_.empty()) {
      {
        std::lock_guard<std::mutex> guard(mutex_);
        feedback.error_code = error_info_.error_code();
        feedback.error_msg = error_info_.error_msg();

        feedback.enemy_x = translation_[0];
        feedback.enemy_y = translation_[1];
        feedback.enemy_z = translation_[2];

        feedback.enemy_roll = rotation_[0];
        feedback.enemy_pitch = rotation_[1];
        feedback.enemy_yaw = rotation_[2];
        as_.publishFeedback(feedback);
        translation_.clear();
        rotation_.clear();
      }
    }
  }
}

void ArmorDetectionNode::ExecuteLoop() {
  std::vector<float> translation;
  std::vector<float> rotation;

  while(running_) {
    if (node_state_ == NodeState::RUNNING) {
      translation.clear();
      rotation.clear();
      ErrorInfo error_info = armor_detector_->DetectArmor(translation, rotation);
      {
        std::lock_guard<std::mutex> guard(mutex_);
        translation_ = translation;
        rotation_ = rotation;
        error_info_ = error_info;
      }

      if (translation.empty() && rotation.empty()) {
        enemy_pos.enemy_dist = 0;
        enemy_pos.enemy_pitch = 0;
        enemy_pos.enemy_yaw = 0;
      } else {
        enemy_pos.enemy_dist = translation[2];
        enemy_pos.enemy_pitch = rotation[1];
        enemy_pos.enemy_yaw = rotation[2];
      }
      PublishMsgs();
      //TODO(noah.guo): coordinat transformation, data fitting.
    } else if (node_state_ == NodeState::PAUSE) {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_var_.wait(lock);
    }
  }
}

void ArmorDetectionNode::PublishMsgs() {
  enemy_info_pub_.publish(enemy_pos);
}

void ArmorDetectionNode::StartThread() {
  LOG_INFO << "Armor detection node started!";
  running_ = true;
  if(node_state_ == NodeState::IDLE) {
    armor_detection_thread_ = std::thread(&ArmorDetectionNode::ExecuteLoop, this);
  }
  node_state_ = NodeState::RUNNING;
  condition_var_.notify_one();
}

void ArmorDetectionNode::PauseThread() {
  LOG_INFO << "Armor detection thread paused!";
  node_state_ = NodeState::PAUSE;
}

void ArmorDetectionNode::StopThread() {
  node_state_ = NodeState::IDLE;
  running_ = false;
  if (armor_detection_thread_.joinable()) {
    armor_detection_thread_.join();
  }
}

ArmorDetectionNode::~ArmorDetectionNode() {
  StopThread();
}
} //namespace detection
} //namespace perception
} //namespace rrts

MAIN(rrts::perception::detection::ArmorDetectionNode, "armor_detection_node")