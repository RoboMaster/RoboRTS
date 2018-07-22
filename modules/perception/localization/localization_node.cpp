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

#include "modules/perception/localization/localization_node.h"

namespace rrts {
namespace perception {
namespace localization {

LocalizationNode::LocalizationNode(std::string name) :
    rrts::common::RRTS::RRTS(name,4) {

  CHECK(Init()) << "Module localization_node initalized failed!";
  initialized_ = true;

}

bool LocalizationNode::Init() {

  std::string prototxt_file_name = "/modules/perception/localization/config/localization.prototxt";
  bool read_state = rrts::common::ReadProtoFromTextFile(prototxt_file_name,&localization_config_);
  if(!read_state){
    LOG_ERROR << "Cannot open " << prototxt_file_name;
    return false;
  }

  tf_broadcaster_ptr_ = new tf::TransformBroadcaster();
  tf_listener_ptr_ = new tf::TransformListener();

  odom_frame_ = localization_config_.odom_frame_id();
  global_frame_ = localization_config_.global_frame_id();
  base_frame_ = localization_config_.base_frame_id();

  init_pose_ << localization_config_.initial_pose_x(),
      localization_config_.initial_pose_y(),
      localization_config_.initial_pose_a();

  init_cov_ << localization_config_.initial_cov_xx(),
      localization_config_.initial_cov_yy(),
      localization_config_.initial_cov_aa();

  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);

  map_sub_ = nh_.subscribe("map", 1, &LocalizationNode::MapCallback, this);

  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>
      (nh_,
       localization_config_.laser_topic_name(),
       100);
  laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                                     *tf_listener_ptr_,
                                                                     odom_frame_,
                                                                     100);
  laser_scan_filter_->registerCallback(boost::bind(&LocalizationNode::LaserScanCallback,
                                                   this, _1));

  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &LocalizationNode::InitialPoseCallback, this);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2, true);

  clean_laser_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("clean_laser_scan", 100, true);

  amcl_ptr_= std::make_unique<Amcl>();
  amcl_ptr_->Init(init_pose_, init_cov_);

  enable_uwb_ = localization_config_.enable_uwb();
  if(enable_uwb_) {
    LOG_INFO << "Enable uwb correction!";
    uwb_frame_ = localization_config_.uwb_frame_id();
    uwb_topic_name_ = localization_config_.uwb_topic_name();
    if(localization_config_.uwb_correction_frequency() > 0) {
      uwb_thread_delay_ = static_cast<int>(1 / localization_config_.uwb_correction_frequency() * 1000);
    } else
    {
      uwb_thread_delay_ = 50.0;
    }
    if(localization_config_.use_sim_uwb()) {
      ground_truth_sub_ = nh_.subscribe("base_pose_ground_truth", 100, &LocalizationNode::GroudTruthCallback, this);
      fake_uwb_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("uwb", 100, true);
    }

    uwb_pose_sub_ = nh_.subscribe("uwb", 10, &LocalizationNode::UwbCallback, this);
    uwb_amcl_thread_ = std::thread(std::bind(&LocalizationNode::UwbAmclThread, this));
  }

  transform_tolerance_.fromSec(localization_config_.transform_tolerance());
  LOG_INFO << "Localization Init!";

  return true;

}


LocalizationNode::~LocalizationNode() {
  uwb_amcl_thread_.join();
  delete laser_scan_filter_;
  delete laser_scan_sub_;
}

void LocalizationNode::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg) {

  if (first_map_only_ && first_map_received_) {
    return;
  }

  LOG_INFO << "Receive First Map";

  amcl_ptr_->HandleMapMessage(*map_msg, init_pose_, init_cov_);
  first_map_received_ = true;

}

void LocalizationNode::UwbCallback(const geometry_msgs::PoseStamped::ConstPtr &uwb_msg) {

  if(!first_map_received_ || !amcl_ptr_->CheckTfUpdate()){
    return;
  }

  ros::Time current_time = ros::Time::now();
  tf::Stamped<tf::Pose> uwb_pose_in_uwb,uwb_pose_in_map;
  tf::poseStampedMsgToTF(*uwb_msg,uwb_pose_in_uwb);
  uwb_pose_in_uwb.stamp_ = ros::Time(0);
  bool error = false;
  try {
    tf_listener_ptr_->transformPose("map",uwb_pose_in_uwb,uwb_pose_in_map);
  } catch (tf::TransformException e) {
    error = true;
    update_uwb_ = false;
    LOG_ERROR << "Uwb Callback TF error: " << e.what();
  }
  if(uwb_init_) {

    math::Vec3d uwb_pose_now;
    uwb_pose_now << uwb_pose_in_map.getOrigin().x(),
        uwb_pose_in_map.getOrigin().y(),
        0;

    auto dt = (current_time - uwb_latest_time);
    if(dt.toSec() > 0) {
      uwb_latest_time = current_time;
      uwb_odom_vel_(0) = (uwb_pose_now(0) - uwb_latest_pose_(0)) / dt.toSec();
      uwb_odom_vel_(1) = (uwb_pose_now(1) - uwb_latest_pose_(1)) / dt.toSec();

      uwb_latest_pose_ = uwb_pose_now;
      update_uwb_ = true;

    }
  } else if(!error){
    LOG_INFO << "UWB Callback Init";
    uwb_latest_pose_  << uwb_pose_in_map.getOrigin().x(),
        uwb_pose_in_map.getOrigin().y(),
        0;
    uwb_latest_time = current_time;
    uwb_init_ = true;
  }
}

void LocalizationNode::UwbAmclThread(){
  while (ros::ok()) {
    while (!first_map_received_ && !uwb_init_ || !amcl_ptr_->CheckTfUpdate()) {
      if(!ros::ok()){
        DLOG_INFO << "Uwb Amcl Thread End";
        return;
      }
      usleep(1);
    }
    if(update_uwb_) {
      amcl_ptr_->UpdateUwb(uwb_latest_pose_ );
      update_uwb_ = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(uwb_thread_delay_));
  }

}

void LocalizationNode::GroudTruthCallback(const nav_msgs::Odometry::ConstPtr &msg){
  geometry_msgs::PoseStamped fake_uwb_pose;
  fake_uwb_pose.header.stamp = ros::Time::now();
  fake_uwb_pose.header.frame_id = "uwb";
  fake_uwb_pose.pose.position.x = msg->pose.pose.position.x + math::RandomGaussianNum<double>(0.1 * 0.2);
  fake_uwb_pose.pose.position.y = msg->pose.pose.position.y + math::RandomGaussianNum<double>(0.1 * 0.2);
  fake_uwb_pose.pose.orientation = msg->pose.pose.orientation;
  fake_uwb_pose_pub_.publish(fake_uwb_pose);
}

void LocalizationNode::InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {

  tf::Pose pose_new;
  TransformInitialPose(pose_new, msg);
  // Transform into the global frame
  DLOG_INFO << "Setting pose " << ros::Time::now().toSec() << ", "
            << pose_new.getOrigin().x() << ", " << pose_new.getOrigin().y();

  math::Vec3d init_pose_mean;
  math::Mat3d init_pose_cov;
  init_pose_mean.setZero();
  init_pose_cov.setZero();

  init_pose_mean(0) = pose_new.getOrigin().x();
  init_pose_mean(1) = pose_new.getOrigin().y();
  init_pose_mean(2) = GetYawFromTfPose(pose_new);

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      init_pose_cov(i, j) = msg->pose.covariance[6 * i + j];
    }
  }
  init_pose_cov(2, 2) = msg->pose.covariance[6 * 5 + 5];

  amcl_ptr_->HandleInitialPoseMessage(init_pose_mean, init_pose_cov);

}

void LocalizationNode::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan_ptr) {

  if (!first_map_received_) {
    return;
  }

  sensor_msgs::LaserScan laser_scan_msg = *laser_scan_ptr;
  laser_msg_time_stamp_ = laser_scan_msg.header.stamp;

  //Check laser init
  if (!laser_init_) {
    math::Vec3d laser_pose_v;
    TransformLaserpose(laser_scan_msg, laser_pose_v);
    amcl_ptr_->SetLaserSensorPose(laser_pose_v);
    laser_init_ = true;
  }

  //Get Odom from TF
  math::Vec3d pose;
  pose.setZero();
  if (!GetOdomPose(latest_odom_pose_, pose,
                   laser_msg_time_stamp_,
                   base_frame_)) {
    LOG_ERROR << "Couldn't determine robot's pose associated with laser scan";
    return;
  }

  double angle_min_in_baseframe;
  double angle_increment_in_baseframe;
  TransformLaserscanToBaseFrame(angle_min_in_baseframe,
                                angle_increment_in_baseframe,
                                laser_scan_msg);

  amcl_ptr_->Update(pose,
                    laser_scan_msg,
                    angle_min_in_baseframe,
                    angle_increment_in_baseframe,
                    particle_cloud_poses_,
                    hyp_pose_);

  sensor_msgs::LaserScan clean_laser_scan_msg = amcl_ptr_->GetCleanLaserScan();
  clean_laser_scan_pub_.publish(clean_laser_scan_msg);

  if (amcl_ptr_->CheckTfUpdate()) {
    PublishUpdatedTF();
  } else if (latest_tf_valid_) {
    PublishLatestValidTF();
  }

  if (amcl_ptr_->CheckParticlePoseCloudPublish()) {
    particle_cloud_poses_.header.stamp = ros::Time::now();
    particle_cloud_poses_.header.frame_id = global_frame_;
    particlecloud_pub_.publish(particle_cloud_poses_);
  }

  if (amcl_ptr_->CheckPosePublish()) {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    TransfromHypPoseToMsg(hyp_pose_, pose_msg);
    pose_pub_.publish(pose_msg);
  }

}

bool LocalizationNode::GetOdomPose(tf::Stamped<tf::Pose> &odom_pose,
                                   math::Vec3d &pose,
                                   const ros::Time &timestamp,
                                   const std::string &frame_id) {
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),
                                            tf::Vector3(0, 0, 0)),
                              timestamp,
                              frame_id);
  try {
    this->tf_listener_ptr_->transformPose(odom_frame_,
                                          ident,
                                          odom_pose);
  }
  catch (tf::TransformException e) {
    LOG_WARNING << "Failed to compute odom pose, skipping scan (" << e.what() << ")";
    return false;
  }
  pose[0] = odom_pose.getOrigin().x();
  pose[1] = odom_pose.getOrigin().y();
  double pitch, roll;
  odom_pose.getBasis().getEulerYPR(pose[2], pitch, roll);
  return true;
}

void LocalizationNode::TransformInitialPose(tf::Pose &pose_new,
                                            const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  if (msg->header.frame_id == "") {
    LOG_WARNING << "Received initial pose with empty frame_id.";
  }
    // Only accept initial pose estimates in the global frame
  else if (tf_listener_ptr_->resolve(msg->header.frame_id) !=
      tf_listener_ptr_->resolve(global_frame_)) {
    LOG_WARNING << "Ignoring initial pose in frame \" "
                << msg->header.frame_id
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
                                       msg->header.stamp,
                                       base_frame_, now,
                                       odom_frame_, ros::Duration(0.5));
    tf_listener_ptr_->lookupTransform(base_frame_, msg->header.stamp,
                                      base_frame_, now,
                                      odom_frame_, tx_odom);
  }
  catch (tf::TransformException e) {
    if (sent_first_transform_) {
      LOG_WARNING << "Failed to transform initial pose in time " << e.what();
    }
    tx_odom.setIdentity();
  }

  tf::Pose pose_old;
  tf::poseMsgToTF(msg->pose.pose, pose_old);
  pose_new = pose_old * tx_odom;

}

bool LocalizationNode::TransformLaserpose(const sensor_msgs::LaserScan &laser_scan_msg,
                                          math::Vec3d &laser_pose) {

  tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),
                                            tf::Vector3(0, 0, 0)),
                              ros::Time(),
                              laser_scan_msg.header.frame_id);
  tf::Stamped<tf::Pose> laser_pose_stamp;
  try {
    this->tf_listener_ptr_->transformPose(base_frame_,
                                          ident,
                                          laser_pose_stamp);
  } catch (tf::TransformException e) {
    LOG_ERROR << "Couldn't transform from "
              << laser_scan_msg.header.frame_id
              << "to "
              << base_frame_;
    return false;
  }

  laser_pose.setZero();
  laser_pose[0] = laser_pose_stamp.getOrigin().x();
  laser_pose[1] = laser_pose_stamp.getOrigin().y();
  DLOG_INFO << "Received laser's pose wrt robot: "<<
      laser_pose[0] << ", " <<
      laser_pose[1] << ", " <<
      laser_pose[2];
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

double LocalizationNode::GetYawFromTfPose(const tf::Pose &tf_pose) {
  double yaw, pitch, roll;
  tf_pose.getBasis().getEulerYPR(yaw, pitch, roll);
  return yaw;
}

void LocalizationNode::TransfromHypPoseToMsg(const HypPose &hyp_pose,
                                             geometry_msgs::PoseWithCovarianceStamped &hyp_pose_msg) {
  hyp_pose_msg.header.frame_id = global_frame_;
  hyp_pose_msg.header.stamp = laser_msg_time_stamp_;
  hyp_pose_msg.pose.pose.position.x = hyp_pose.pose_mean[0];
  hyp_pose_msg.pose.pose.position.y = hyp_pose.pose_mean[1];
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyp_pose.pose_mean[2]),
                        hyp_pose_msg.pose.pose.orientation);

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      hyp_pose_msg.pose.covariance[6 * i + j] = hyp_pose.pose_set_cov(i, j);
    }
  }
  hyp_pose_msg.pose.covariance[6 * 5 + 5] = hyp_pose.pose_set_cov(2, 2);
}

bool LocalizationNode::PublishUpdatedTF() {
  tf::Stamped<tf::Pose> odom_to_map;
  try {
    tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyp_pose_.pose_mean[2]),
                         tf::Vector3(hyp_pose_.pose_mean[0],
                                     hyp_pose_.pose_mean[1],
                                     0.0));
    tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(),
                                         laser_msg_time_stamp_,
                                         localization_config_.base_frame_id());
    this->tf_listener_ptr_->transformPose(odom_frame_,
                                          tmp_tf_stamped,
                                          odom_to_map);
  } catch (tf::TransformException e) {
    LOG_ERROR << "Failed to subtract base to odom transform" << e.what();
    return -1;
  }

  latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                             tf::Point(odom_to_map.getOrigin()));
  latest_tf_valid_ = true;

  ros::Time transform_expiration = (laser_msg_time_stamp_
      + transform_tolerance_);
  tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                      transform_expiration,
                                      global_frame_,
                                      odom_frame_);
  this->tf_broadcaster_ptr_->sendTransform(tmp_tf_stamped);
  sent_first_transform_ = true;
}

void LocalizationNode::PublishLatestValidTF() {
  ros::Time transform_expiration = (laser_msg_time_stamp_
      + transform_tolerance_);
  tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                      transform_expiration,
                                      global_frame_,
                                      odom_frame_);

  this->tf_broadcaster_ptr_->sendTransform(tmp_tf_stamped);
}


}
}
}

MAIN(rrts::perception::localization::LocalizationNode,"LocalizationNode");