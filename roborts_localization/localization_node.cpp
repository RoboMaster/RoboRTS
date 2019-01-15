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

#include "localization_node.h"

namespace roborts_localization{

LocalizationNode::LocalizationNode(std::string name) {
  CHECK(Init()) << "Module "  << name <<" initialized failed!";
  initialized_ = true;
}


bool LocalizationNode::Init() {

  LocalizationConfig localization_config;
  localization_config.GetParam(&nh_);

  odom_frame_   = std::move(localization_config.odom_frame_id);
  global_frame_ = std::move(localization_config.global_frame_id);
  base_frame_   = std::move(localization_config.base_frame_id);

  laser_topic_ = std::move(localization_config.laser_topic_name);

  init_pose_ = {localization_config.initial_pose_x,
                localization_config.initial_pose_y,
                localization_config.initial_pose_a};
  init_cov_ = {localization_config.initial_cov_xx,
               localization_config.initial_cov_yy,
               localization_config.initial_cov_aa};

  transform_tolerance_  = ros::Duration(localization_config.transform_tolerance);
  publish_visualize_ = localization_config.publish_visualize;

  tf_broadcaster_ptr_ = std::make_unique<tf::TransformBroadcaster>();
  tf_listener_ptr_ = std::make_unique<tf::TransformListener>();

  distance_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("distance_map", 1, true);
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);

  // Use message filter for time synchronizer (laser scan topic and tf between odom and base frame)
  laser_scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_, laser_topic_, 100);
  laser_scan_filter_ = std::make_unique<tf::MessageFilter<sensor_msgs::LaserScan>>(*laser_scan_sub_,
                                                                                   *tf_listener_ptr_,
                                                                                   odom_frame_,
                                                                                   100);
  laser_scan_filter_->registerCallback(boost::bind(&LocalizationNode::LaserScanCallback, this, _1));

  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &LocalizationNode::InitialPoseCallback, this);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("amcl_pose", 2, true);

  amcl_ptr_= std::make_unique<Amcl>();
  amcl_ptr_->GetParamFromRos(&nh_);
  amcl_ptr_->Init(init_pose_, init_cov_);

  map_init_ = GetStaticMap();
  laser_init_ = GetLaserPose();

  return map_init_&&laser_init_;
}

bool LocalizationNode::GetStaticMap(){
  static_map_srv_ = nh_.serviceClient<nav_msgs::GetMap>("static_map");
  ros::service::waitForService("static_map", -1);
  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;
  if(static_map_srv_.call(req,res)) {
    LOG_INFO << "Received Static Map";
    amcl_ptr_->HandleMapMessage(res.map, init_pose_, init_cov_);
    first_map_received_ = true;
    return true;
  } else{
    LOG_ERROR << "Get static map failed";
    return false;
  }
}

bool LocalizationNode::GetLaserPose() {
  auto laser_scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(laser_topic_);

  Vec3d laser_pose;
  laser_pose.setZero();
  GetPoseFromTf(base_frame_, laser_scan_msg->header.frame_id, ros::Time(), laser_pose);
  laser_pose[2] = 0; // No need for rotation, or will be error
  DLOG_INFO << "Received laser's pose wrt robot: "<<
            laser_pose[0] << ", " <<
            laser_pose[1] << ", " <<
            laser_pose[2];

  amcl_ptr_->SetLaserSensorPose(laser_pose);
  return true;
}

void LocalizationNode::InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &init_pose_msg) {

  if (init_pose_msg->header.frame_id == "") {
    LOG_WARNING << "Received initial pose with empty frame_id.";
  } // Only accept initial pose estimates in the global frame
  else if (tf_listener_ptr_->resolve(init_pose_msg->header.frame_id) !=
           tf_listener_ptr_->resolve(global_frame_)) {
    LOG_ERROR << "Ignoring initial pose in frame \" "
              << init_pose_msg->header.frame_id
              << "\"; initial poses must be in the global frame, \""
              << global_frame_;
    return;
  }

  // In case the client sent a pose estimate in the past, integrate the
  // intervening odometric change.
  tf::StampedTransform tx_odom;
  try {
    ros::Time now = ros::Time::now();
    tf_listener_ptr_->waitForTransform(base_frame_,
                                       init_pose_msg->header.stamp,
                                       base_frame_,
                                       now,
                                       odom_frame_,
                                       ros::Duration(0.5));
    tf_listener_ptr_->lookupTransform(base_frame_,
                                      init_pose_msg->header.stamp,
                                      base_frame_,
                                      now,
                                      odom_frame_, tx_odom);
  }
  catch (tf::TransformException &e) {
    tx_odom.setIdentity();
  }
  tf::Pose pose_new;
  tf::Pose pose_old;
  tf::poseMsgToTF(init_pose_msg->pose.pose, pose_old);
  pose_new = pose_old * tx_odom;

  // Transform into the global frame
  DLOG_INFO << "Setting pose " << ros::Time::now().toSec() << ", "
            << pose_new.getOrigin().x() << ", " << pose_new.getOrigin().y();

  Vec3d init_pose_mean;
  Mat3d init_pose_cov;
  init_pose_mean.setZero();
  init_pose_cov.setZero();
  double yaw, pitch, roll;
  init_pose_mean(0) = pose_new.getOrigin().x();
  init_pose_mean(1) = pose_new.getOrigin().y();
  pose_new.getBasis().getEulerYPR(yaw, pitch, roll);
  init_pose_mean(2) = yaw;
  init_pose_cov = math::MsgCovarianceToMat3d(init_pose_msg->pose.covariance);

  amcl_ptr_->HandleInitialPoseMessage(init_pose_mean, init_pose_cov);
}

void LocalizationNode::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg_ptr){

  last_laser_msg_timestamp_ = laser_scan_msg_ptr->header.stamp;

  Vec3d pose_in_odom;
  if(!GetPoseFromTf(odom_frame_, base_frame_, last_laser_msg_timestamp_, pose_in_odom))
  {
    LOG_ERROR << "Couldn't determine robot's pose";
    return;
  }

  double angle_min = 0 , angle_increment = 0;
  sensor_msgs::LaserScan laser_scan_msg = *laser_scan_msg_ptr;
  TransformLaserscanToBaseFrame(angle_min, angle_increment, laser_scan_msg);

  amcl_ptr_->Update(pose_in_odom,
                    laser_scan_msg,
                    angle_min,
                    angle_increment,
                    particlecloud_msg_,
                    hyp_pose_);

  LOG_ERROR_IF(!PublishTf()) << "Publish Tf Error!";

  if(publish_visualize_){
    PublishVisualize();
  }

}

void LocalizationNode::PublishVisualize(){

  if(pose_pub_.getNumSubscribers() > 0){
    pose_msg_.header.stamp = ros::Time::now();
    pose_msg_.header.frame_id = global_frame_;
    pose_msg_.pose.position.x = hyp_pose_.pose_mean[0];
    pose_msg_.pose.position.y = hyp_pose_.pose_mean[1];
    pose_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(hyp_pose_.pose_mean[2]);
    pose_pub_.publish(pose_msg_);
  }

  if(particlecloud_pub_.getNumSubscribers() > 0){
    particlecloud_msg_.header.stamp = ros::Time::now();
    particlecloud_msg_.header.frame_id = global_frame_;
    particlecloud_pub_.publish(particlecloud_msg_);
  }

  if(!publish_first_distance_map_) {
    distance_map_pub_.publish(amcl_ptr_->GetDistanceMapMsg());
    publish_first_distance_map_ = true;
  }
}

bool LocalizationNode::PublishTf() {
  ros::Time transform_expiration = (last_laser_msg_timestamp_ + transform_tolerance_);
  if (amcl_ptr_->CheckTfUpdate()) {
    // Subtracting base to odom from map to base and send map to odom instead
    tf::Stamped<tf::Pose> odom_to_map;
    try {
      tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyp_pose_.pose_mean[2]),
                           tf::Vector3(hyp_pose_.pose_mean[0],
                                       hyp_pose_.pose_mean[1],
                                       0.0));
      tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(),
                                           last_laser_msg_timestamp_,
                                           base_frame_);
      this->tf_listener_ptr_->transformPose(odom_frame_,
                                            tmp_tf_stamped,
                                            odom_to_map);
    } catch (tf::TransformException &e) {
      LOG_ERROR << "Failed to subtract base to odom transform" << e.what();
      return false;
    }

    latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                               tf::Point(odom_to_map.getOrigin()));
    latest_tf_valid_ = true;

    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        transform_expiration,
                                        global_frame_,
                                        odom_frame_);
    this->tf_broadcaster_ptr_->sendTransform(tmp_tf_stamped);
    sent_first_transform_ = true;
    return true;
  } else if (latest_tf_valid_) {
    // Nothing changed, so we'll just republish the last transform
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        transform_expiration,
                                        global_frame_,
                                        odom_frame_);
    this->tf_broadcaster_ptr_->sendTransform(tmp_tf_stamped);
    return true;
  }
  else{
    return false;
  }
}

bool LocalizationNode::GetPoseFromTf(const std::string &target_frame,
                   const std::string &source_frame,
                   const ros::Time &timestamp,
                   Vec3d &pose)
{
  tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),
                                            tf::Vector3(0, 0, 0)),
                              timestamp,
                              source_frame);
  tf::Stamped<tf::Pose> pose_stamp;
  try {
    this->tf_listener_ptr_->transformPose(target_frame,
                                          ident,
                                          pose_stamp);
  } catch (tf::TransformException &e) {
    LOG_ERROR << "Couldn't transform from "
              << source_frame
              << "to "
              << target_frame;
    return false;
  }

  pose.setZero();
  pose[0] = pose_stamp.getOrigin().x();
  pose[1] = pose_stamp.getOrigin().y();
  double yaw,pitch, roll;
  pose_stamp.getBasis().getEulerYPR(yaw, pitch, roll);
  pose[2] = yaw;
  return true;
}

void LocalizationNode::TransformLaserscanToBaseFrame(double &angle_min,
                                                     double &angle_increment,
                                                     const sensor_msgs::LaserScan &laser_scan_msg) {

  // To account for lasers that are mounted upside-down, we determine the
  // min, max, and increment angles of the laser in the base frame.
  // Construct min and max angles of laser, in the base_link frame.
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, laser_scan_msg.angle_min);
  tf::Stamped<tf::Quaternion> min_q(q, laser_scan_msg.header.stamp,
                                    laser_scan_msg.header.frame_id);
  q.setRPY(0.0, 0.0, laser_scan_msg.angle_min
      + laser_scan_msg.angle_increment);
  tf::Stamped<tf::Quaternion> inc_q(q, laser_scan_msg.header.stamp,
                                    laser_scan_msg.header.frame_id);

  try {
    tf_listener_ptr_->transformQuaternion(base_frame_,
                                          min_q,
                                          min_q);
    tf_listener_ptr_->transformQuaternion(base_frame_,
                                          inc_q,
                                          inc_q);
  }
  catch (tf::TransformException &e) {
    LOG_WARNING << "Unable to transform min/max laser angles into base frame: " << e.what();
    return;
  }

  angle_min = tf::getYaw(min_q);
  angle_increment = (tf::getYaw(inc_q) - angle_min);

  // Wrapping angle to [-pi .. pi]
  angle_increment = (std::fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI);

}

}// roborts_localization

int main(int argc, char **argv) {
  roborts_localization::GLogWrapper glog_wrapper(argv[0]);
  ros::init(argc, argv, "localization_node");
  roborts_localization::LocalizationNode localization_node("localization_node");
  ros::AsyncSpinner async_spinner(THREAD_NUM);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}

