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

#include <unistd.h>
#include "modules/perception/detection/armor_detection/armor_detection_node.h"

namespace rrts{
namespace perception {
namespace detection {

ArmorDetectionNode::ArmorDetectionNode(std::string name):
    node_state_(rrts::common::IDLE),
    demensions_(3),
    initialized_(false),
    detected_enemy_(false),
    undetected_count_(0),
    rrts::common::RRTS::RRTS(name),
    as_(nh_, name+"_action", boost::bind(&ArmorDetectionNode::ActionCB, this, _1), false) {
  initialized_ = false;
  enemy_nh_ = ros::NodeHandle("/constraint_set");
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

ErrorInfo ArmorDetectionNode::Init() {
  enemy_info_pub_ = enemy_nh_.advertise<messages::EnemyPos>("enemy_pos", 100);

  ArmorDetectionAlgorithms armor_detection_algorithms;
  std::string file_name = "/modules/perception/detection/armor_detection/config/armor_detection.prototxt";
  bool read_state = rrts::common::ReadProtoFromTextFile(file_name, &armor_detection_algorithms);
  if (!read_state) {
    LOG_ERROR << "Cannot open " << file_name;
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  }

  //create the selected algorithms
  std::string selected_algorithm = armor_detection_algorithms.selected_algorithm();
  armor_detector_ = rrts::common::AlgorithmFactory<ArmorDetectionBase>::CreateAlgorithm(selected_algorithm);
  max_rotating_fps_ = armor_detection_algorithms.max_rotating_fps();
  min_rotating_detected_count_ = armor_detection_algorithms.min_rotating_detected_count();
  undetected_armor_delay_ = armor_detection_algorithms.undetected_armor_delay();

  if (armor_detector_ == nullptr) {
    LOG_ERROR << "Create armor_detector_ pointer failed!";
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  } else
    return ErrorInfo(ErrorCode::OK);
}

void ArmorDetectionNode::ActionCB(const messages::ArmorDetectionGoal::ConstPtr &data) {
  messages::ArmorDetectionFeedback feedback;
  messages::ArmorDetectionResult result;
  bool undetected_msg_published = false;

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
  ros::Rate rate(25);
  while(ros::ok()) {

    if(as_.isPreemptRequested()) {
      as_.setPreempted();
      return;
    }

    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (undetected_count_ != 0) {
        feedback.detected = true;
        feedback.error_code = error_info_.error_code();
        feedback.error_msg = error_info_.error_msg();

        feedback.enemy_pos.header.frame_id = "camera0";
        feedback.enemy_pos.header.stamp    = ros::Time::now();

        feedback.enemy_pos.pose.position.x = x_;
        feedback.enemy_pos.pose.position.y = y_;
        feedback.enemy_pos.pose.position.z = z_;
        feedback.enemy_pos.pose.orientation.w = 1;
        as_.publishFeedback(feedback);
        undetected_msg_published = false;
      } else if(!undetected_msg_published) {
        feedback.detected = false;
        feedback.error_code = error_info_.error_code();
        feedback.error_msg = error_info_.error_msg();

        feedback.enemy_pos.header.frame_id = "camera0";
        feedback.enemy_pos.header.stamp    = ros::Time::now();

        feedback.enemy_pos.pose.position.x = 0;
        feedback.enemy_pos.pose.position.y = 0;
        feedback.enemy_pos.pose.position.z = 0;
        feedback.enemy_pos.pose.orientation.w = 1;
        as_.publishFeedback(feedback);
        undetected_msg_published = true;
      }
    }
    rate.sleep();
  }
}

void ArmorDetectionNode::ExecuteLoop() {
  double x, y, z, dis, pitch, yaw;
  unsigned int enemy_is_rotating = 0;
  undetected_count_ = undetected_armor_delay_;
  std::vector<int> fps;
  double center_x = 0, center_y = 0, center_z = 0, center_dis = 0, center_pitch = 0;

  unsigned int rotating_detected_count = 0;
  while(running_) {
    usleep(1);
    if (node_state_ == NodeState::RUNNING) {
      ErrorInfo error_info = armor_detector_->DetectArmor(detected_enemy_, x, y, z, dis, pitch, yaw);
      {
        std::lock_guard<std::mutex> guard(mutex_);
        x_ = x;
        y_ = y;
        z_ = z;
        error_info_ = error_info;
      }
      int tmp = detected_enemy_ ? 1 : -1;
      if(fps.size() < max_rotating_fps_)
        fps.push_back(tmp);
      else{
        fps.push_back(tmp);
        fps.erase(fps.begin());
        for(unsigned int i = 0; i < fps.size() - 1; i++) {
          rotating_detected_count += fps[i]*fps[i+1] < 0 ? 1 : 0;
          std::cout << fps[i]*fps[i+1];
        }
        std::cout << std::endl;
        std::cout << "**********************************" << std::endl;
        std::cout << "count: " << rotating_detected_count << std::endl;
        if(rotating_detected_count > min_rotating_detected_count_ || enemy_is_rotating != 0) {
          if(enemy_is_rotating == 0 || rotating_detected_count > min_rotating_detected_count_)
            enemy_is_rotating = 1;
          enemy_is_rotating--;
          std::lock_guard<std::mutex> guard(mutex_);
          x_ = center_x; y_ = center_y; z_ = center_z;
          rotating_detected_count = 0;
          undetected_count_ = undetected_armor_delay_;
          enemy_pos.enemy_pos.header.frame_id = "camera0";
          enemy_pos.enemy_pos.header.stamp    = ros::Time::now();
          enemy_pos.enemy_pos.pose.position.x = center_x;
          enemy_pos.enemy_pos.pose.position.y = center_y;
          enemy_pos.enemy_pos.pose.position.z = center_z;
          enemy_pos.enemy_pos.pose.orientation.w = 1;

          enemy_pos.dis = center_dis;
          enemy_pos.pitch = center_pitch;
          enemy_pos.yaw = 0;
          PublishMsgs();
          continue;
        }
        rotating_detected_count = 0;
      }

      if(detected_enemy_) {
        enemy_pos.enemy_pos.header.frame_id = "camera0";
        enemy_pos.enemy_pos.header.stamp    = ros::Time::now();
        enemy_pos.enemy_pos.pose.position.x = x;
        enemy_pos.enemy_pos.pose.position.y = y;
        enemy_pos.enemy_pos.pose.position.z = z;
        enemy_pos.enemy_pos.pose.orientation.w = 1;

        enemy_pos.dis = dis;
        enemy_pos.pitch = pitch;
        enemy_pos.yaw = yaw;

        if(rotating_detected_count <= min_rotating_detected_count_) {
          center_x = x; center_y = y; center_z = z;
          center_dis = dis; center_pitch = pitch;
        }
        std::lock_guard<std::mutex> guard(mutex_);
        undetected_count_ = undetected_armor_delay_;
        PublishMsgs();
      } else if(undetected_count_ != 0) {
        enemy_pos.enemy_pos.header.frame_id = "camera0";
        enemy_pos.enemy_pos.header.stamp    = ros::Time::now();
        enemy_pos.enemy_pos.pose.position.x = x;
        enemy_pos.enemy_pos.pose.position.y = y;
        enemy_pos.enemy_pos.pose.position.z = z;
        enemy_pos.enemy_pos.pose.orientation.w = 1;

        enemy_pos.dis = dis;
        enemy_pos.pitch = pitch;
        enemy_pos.yaw = 0;

        undetected_count_--;
        PublishMsgs();
      }
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
