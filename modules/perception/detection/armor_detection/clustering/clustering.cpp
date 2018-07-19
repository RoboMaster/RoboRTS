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
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include "modules/perception/detection/armor_detection/clustering/clustering.h"

#include "common/timer.h"
#include "common/io.h"
#include "common/log.h"

namespace rrts{
namespace perception {
namespace detection {

ClusteringNode::ClusteringNode(std::string name):rrts::common::RRTS::RRTS(name) {
  //LoadParam();
  tf_listener_ = std::make_shared<tf::TransformListener>(ros::Duration(1));
  nh_scan_ = ros::NodeHandle();
  nh_ = ros::NodeHandle("/clustering");
  enemy_pos_sub_ = nh_.subscribe("/constraint_set/enemy_pos", 2, &ClusteringNode::EnemyPoseCallBack, this);
  scan_sub_ = nh_scan_.subscribe("scan", 20, &ClusteringNode::ScanCallBack, this);
  enemy_pos_pub_ = nh_.advertise<messages::EnemyPos>("enemy_pos", 10);
  error_info_ = ErrorInfo(rrts::common::OK);
  std::cout<<"cluster begin!"<<std::endl;
}


void ClusteringNode::LoadParam() {
//  //read parameters
//  Clustering clustering_config_;
//  std::string file_name =
//      "/modules/perception/detection/armor_detection/clustering/config/clustering.prototxt";
//  bool read_state = rrts::common::ReadProtoFromTextFile(file_name, &clustering_config_);
//  CHECK(read_state) << "Cannot open " << file_name;
}

ErrorInfo ClusteringNode::DetectArmor(double &x, double &y, double &z, double &distance, double &pitch, double &yaw) {
  return ErrorInfo();
}

void ClusteringNode::EnemyPoseCallBack(const messages::EnemyPosConstPtr &msg) {
  std::lock_guard<std::mutex> guard(mutex_);

  tf::Stamped<tf::Pose> pose_in, pose_out;
  poseStampedMsgToTF(msg->enemy_pos, pose_in);
  pose_in.stamp_ = ros::Time(0);
  try {
    tf_listener_->transformPose("base_laser_link", pose_in, pose_out);
  } catch(tf::TransformException& ex) {
      ROS_WARN("Transformation failed!");
  }
  double roll, pitch;
  pose_out.getBasis().getEulerYPR(enemy_yaw_, pitch, roll);
}

void ClusteringNode::ScanCallBack(const sensor_msgs::LaserScan::ConstPtr &msg) {
  std::lock_guard<std::mutex> guard(mutex_);
  int min_index = static_cast<int>((enemy_yaw_ + M_PI)*180/M_PI + 0.5);

  //判断点是否在障碍物上，然后聚类
  double min_value = msg->ranges[min_index];
  for (int i = -2; i < 3; i ++) {
    int index = (min_index + i)%360;
    if(msg->ranges[index] < min_value) {
      min_value = msg->ranges[index];
      min_index = index;
    }
  }
  enemy_pos_.enemy_pos.header.frame_id = "lidar";
  enemy_pos_.enemy_pos.header.stamp    = ros::Time::now();
  enemy_pos_.enemy_pos.pose.position.x = min_value*std::cos(min_index*M_PI/180.);
  enemy_pos_.enemy_pos.pose.position.y = min_value*std::sin(min_index*M_PI/180.);
  enemy_pos_.enemy_pos.pose.position.z = 10;
  enemy_pos_.enemy_pos.pose.orientation.w = 1;
  
  std::cout << "min_index: " << min_index << std::endl;
  enemy_pos_.yaw = min_index*M_PI/180.;
  enemy_pos_.dis = min_value;
  enemy_pos_pub_.publish(enemy_pos_);
}

ClusteringNode::~ClusteringNode() {

}
} //namespace detection
} //namespace perception
} //namespace rrts
MAIN(rrts::perception::detection::ClusteringNode, "clustering_node")
