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
#include "modules/driver/self_check/self_check_node.h"
#include "modules/driver/self_check/proto/self_check.pb.h"
#include "common/io.h"

namespace rrts{
namespace driver {
namespace selfcheck {

SelfCheckNode::SelfCheckNode(std::string name):
    rrts::common::RRTS::RRTS(name),
    camera_status_(false),
    laser_status_(false),
    imu_status_(false),
    odom_count_(0)
{
  if (!Init().IsOK()) {
    LOG_ERROR << "self_check_node initalized failed!";
  }
  NOTICE("Waiting for input command in self_check_client...")
}

ErrorInfo SelfCheckNode::Init(){
  ros::NodeHandle check_nh;
  check_nh.setCallbackQueue(&self_check_queue_);
  if(!LoadParams()) {
    return ErrorInfo(rrts::common::ErrorCode::Error);
  }
  rrts::driver::camera::CameraParam camera_param;
  camera_param.LoadCameraParam();
  auto camera_param_vec = camera_param.GetCameraParam();
  camera_status_vec_.resize(camera_param_vec.size(), false);
  subs_.resize(camera_param_vec.size());
  it_ = new image_transport::ImageTransport(check_nh);

  for (unsigned int i = 0; i < subs_.size(); i++) {
    std::string topic_name = "camera_" + std::to_string(i);
    subs_[i] = it_->subscribe(topic_name, 1, boost::bind(&SelfCheckNode::CheckCamera, this, _1, i));
  }

  laser_scan_sub_ = check_nh.subscribe("scan", 1, &SelfCheckNode::CheckLaser, this);
  odom_sub_ = check_nh.subscribe("odom",1, &SelfCheckNode::CheckStaticIMU, this);

  ros::NodeHandle nh;
  self_check_srv_ = nh.advertiseService("self_check", &SelfCheckNode::SelfCheckCallback,this);
  check_status_srv_ = nh.serviceClient<messages::CheckStatus>("check_status");
  return ErrorInfo(rrts::common::ErrorCode::OK);
}

bool SelfCheckNode::LoadParams() {
  rrts::driver::selfcheck::SelfCheckConfig self_check_param;
  std::string file_name = "/modules/driver/self_check/config/self_check.prototxt";
  bool read_state = rrts::common::ReadProtoFromTextFile(file_name, &self_check_param);
  if (!read_state) {
    LOG_ERROR << "Cannot open " << file_name;
    return false;
  }
  check_duration_ = ros::Duration(self_check_param.check_duration());
  return true;
}
bool SelfCheckNode::SelfCheckCallback(messages::SelfCheck::Request  &req,
                                      messages::SelfCheck::Response &res){
  if (req.self_check == true) {
    LOG_INFO<<"start self check!";
    //Reset all check state to false
    for(int i = 0; i < camera_status_vec_.size()-1; i++){
      camera_status_vec_[i] = false;
    }
    laser_status_ = false;
    imu_status_ = false;
    camera_status_ = false;
    odom_count_ = 0;

    //initial check time
    initial_check_time_ = ros::Time::now();
    ros::Rate rate(50);
    while (ros::ok() && ros::Time::now()-initial_check_time_<check_duration_){
      self_check_queue_.callOne(ros::WallDuration());
      rate.sleep();
    }
    for(int i = 0; i < camera_status_vec_.size()-1; i++){
      if(camera_status_vec_[i] == false){
        camera_status_ = false;
        break;
      }
    }
    bool self_check_passed = laser_status_ && imu_status_ && camera_status_;
    res.passed = self_check_passed;
    
    messages::CheckStatus check_status_msg;
    check_status_msg.request.self_check_passed = self_check_passed;
    if(check_status_srv_.call(check_status_msg)) {
      LOG_INFO << "Check status" << self_check_passed <<" has send to serial com successfully!";
    } else
      LOG_ERROR << "Check status failed to send to serial com!";
    return true;
  }
  return false;
}

void SelfCheckNode::CheckLaser(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg) {
  if(laser_scan_msg->ranges.size() != 0){
    laser_status_ = true;
    LOG_INFO << "Laser Scan check passed!";
  } else{
    laser_status_ = false;
    LOG_ERROR << "Laser Scan check failed!";
  }
}

void SelfCheckNode::CheckCamera(const sensor_msgs::ImageConstPtr &msg, unsigned int camera_id) {
  if(cv_bridge::toCvShare(msg, "bgr8")->image.empty()){
    camera_status_vec_[camera_id] = false;
    LOG_ERROR << "Camera " << camera_id << "check failed!";
  } else{
    camera_status_vec_[camera_id] = true;
    LOG_INFO << "Camera " << camera_id << " check passed.";
  }
}

void SelfCheckNode::CheckStaticIMU(const nav_msgs::Odometry::ConstPtr& odom_msgs) {
  if (odom_count_ == 0) {
    init_odom_ = *odom_msgs;
  }
  else{
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(init_odom_.pose.pose.orientation, rot1);
    tf::quaternionMsgToTF(odom_msgs->pose.pose.orientation, rot2);
    if (rot1.angleShortestPath(rot2) <0.1){
      imu_status_ = true;
      LOG_INFO << "IMU check passed!";
    } else{
      imu_status_ = false;
      LOG_ERROR<<"angle diff: "<<rot1.angleShortestPath(rot2);
      LOG_ERROR << "IMU check failed!";
    }}
  odom_count_++;
}

SelfCheckNode::~SelfCheckNode() {
  delete(it_);
}

} //namespace selfcheck
} //namespace driver
} //namespace rrts

MAIN(rrts::driver::selfcheck::SelfCheckNode, "self_check_node")

