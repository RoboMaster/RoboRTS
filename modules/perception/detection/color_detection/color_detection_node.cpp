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
#include "modules/perception/detection/color_detection/color_detection_node.h"

namespace rrts{
namespace perception {
namespace detection {

ColorDetectionNode::ColorDetectionNode(std::string name):
    node_state_(rrts::common::IDLE),
    enemy_direction_(0),
    detected_enemy_(false),
    initialized_(false),
    camera_id_(1),
    rrts::common::RRTS::RRTS(name),
    as_(nh_, name+"_action", boost::bind(&ColorDetectionNode::ActionCB, this, _1), false) {
  initialized_ = false;
  if (Init().IsOK()) {
    initialized_ = true;
    node_state_ = rrts::common::IDLE;
  } else {
    LOG_ERROR << "color_detection_node initalized failed!";
    node_state_ = rrts::common::FAILURE;
  }
  NOTICE("Waiting for input command in color_detection_client...")
  as_.start();
}

ErrorInfo ColorDetectionNode::Init() {
  ColorDetectionParams color_detection_params;
  std::string file_name = "/modules/perception/detection/color_detection/config/color_detection.prototxt";
  bool read_state = rrts::common::ReadProtoFromTextFile(file_name, &color_detection_params);
  if (!read_state) {
    LOG_ERROR << "Cannot open " << file_name;
    return ErrorInfo(rrts::common::ErrorCode::DETECTION_INIT_ERROR);
  }
  enemy_color_ = color_detection_params.enemy_color();
  enable_debug_= color_detection_params.enable_debug();
  using_hsv_   = color_detection_params.using_hsv();
  threshold_   = color_detection_params.threshold();
  min_pixel_number_ = color_detection_params.min_pixel_number();
  return ErrorInfo(rrts::common::ErrorCode::OK);
}

void ColorDetectionNode::ActionCB(const messages::EnemyDirectionGoal::ConstPtr &data) {
  messages::EnemyDirectionFeedback feedback;
  messages::EnemyDirectionResult result;
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
  ros::Rate loop(25);
  while(ros::ok()) {
    loop.sleep();
    if(as_.isPreemptRequested()) {
      as_.setPreempted();
      return;
    }

    if (detected_enemy_) {
      std::lock_guard<std::mutex> guard(mutex_);
      feedback.detected = true;
      feedback.error_code = error_info_.error_code();
      feedback.error_msg = error_info_.error_msg();

      feedback.direction = enemy_direction_;
      as_.publishFeedback(feedback);
      undetected_msg_published = false;
    } else if(!undetected_msg_published) {
      std::lock_guard<std::mutex> guard(mutex_);
      feedback.detected = false;
      feedback.error_code = error_info_.error_code();
      feedback.error_msg = error_info_.error_msg();

      feedback.direction = 0;
      as_.publishFeedback(feedback);
      undetected_msg_published = true;
    }
  }
}

void ColorDetectionNode::ExecuteLoop() {
  while(running_) {
    usleep(1);
    if (node_state_ == NodeState::RUNNING) {
      cv_toolbox_.NextImage(src_, camera_id_);
      if (!src_.empty()) {
        NOTICE("Begin to detect enemy!")
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::dilate(src_, src_, element, cv::Point(-1, -1), 1);
        auto binary_color_img = cv_toolbox_.DistillationColor(src_, enemy_color_, using_hsv_);
        cv::threshold(binary_color_img, binary_color_img, threshold_, 255, CV_THRESH_BINARY);
        int count = PixelSum(binary_color_img);
        if(enable_debug_) {
          std::cout << "number: " << count << std::endl;
          cv::imshow("src", binary_color_img);
          cv::waitKey(1);
        }
        if(count > min_pixel_number_) {
          detected_enemy_ = true;
          enemy_direction_ = 1;
        } else {
          detected_enemy_ = false;
          enemy_direction_ = 0;
        }
      } else {
        NOTICE("Waiting for run camera driver...")
        usleep(1000);
      }
    } else if (node_state_ == NodeState::PAUSE) {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_var_.wait(lock);
    }
  }
}

int ColorDetectionNode::PixelSum(cv::Mat src) {
  int counter = 0;
  cv::Mat_<uchar>::iterator it = src.begin<uchar>();
  cv::Mat_<uchar>::iterator itend = src.end<uchar>();
  for (; it!=itend; ++it)
  {
    if((*it)>0) counter+=1;
  }
  return counter;
}

void ColorDetectionNode::StartThread() {
  LOG_INFO << "Armor detection node started!";
  running_ = true;
  if(node_state_ == NodeState::IDLE) {
    armor_detection_thread_ = std::thread(&ColorDetectionNode::ExecuteLoop, this);
  }
  node_state_ = NodeState::RUNNING;
  condition_var_.notify_one();
}

void ColorDetectionNode::PauseThread() {
  LOG_INFO << "Armor detection thread paused!";
  node_state_ = NodeState::PAUSE;
}

void ColorDetectionNode::StopThread() {
  node_state_ = NodeState::IDLE;
  running_ = false;
  if (armor_detection_thread_.joinable()) {
    armor_detection_thread_.join();
  }
}

ColorDetectionNode::~ColorDetectionNode() {
  StopThread();
}
} //namespace detection
} //namespace perception
} //namespace rrts

MAIN(rrts::perception::detection::ColorDetectionNode, "color_detection_node")
