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

#include "chassis.h"
#include "../roborts_sdk/sdk.h"

namespace roborts_base{
Chassis::Chassis(std::shared_ptr<roborts_sdk::Handle> handle):
    handle_(handle){
  SDK_Init();
  ROS_Init();
}
void Chassis::SDK_Init(){
  handle_->CreateSubscriber<roborts_sdk::p_chassis_info_t>(CHASSIS_CMD_SET, PUSH_CHASSIS_INFO,
                                              CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                              std::bind(&Chassis::ChassisInfoCallback, this, std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::p_uwb_data_t>(CHASSIS_CMD_SET, PUSH_UWB_INFO,
                                          CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                          std::bind(&Chassis::UWBInfoCallback, this, std::placeholders::_1));

  chassis_speed_pub_ = handle_->CreatePublisher<roborts_sdk::p_chassis_speed_t>(CHASSIS_CMD_SET, CTRL_CHASSIS_SPEED,
                                                                   MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
  chassis_mode_pub_ = handle_->CreatePublisher<roborts_sdk::chassis_mode_e>(CHASSIS_CMD_SET, SET_CHASSIS_MODE,
                                                               MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
  chassis_spd_acc_pub_ = handle_->CreatePublisher<roborts_sdk::p_chassis_spd_acc_t>(CHASSIS_CMD_SET, CTRL_CHASSIS_SPEED_ACC,
                                                               MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);

}
void Chassis::ROS_Init(){
  //ros publisher
  ros_odom_pub_ = ros_nh_.advertise<nav_msgs::Odometry>("odom", 30);
  ros_uwb_pub_ = ros_nh_.advertise<geometry_msgs::PoseStamped>("uwb", 30);
  //ros subscriber
  ros_sub_cmd_chassis_vel_ = ros_nh_.subscribe("cmd_vel", 1, &Chassis::ChassisSpeedCtrlCallback, this);
  ros_sub_cmd_chassis_vel_acc_ = ros_nh_.subscribe("cmd_vel_acc", 1, &Chassis::ChassisSpeedAccCtrlCallback, this);
  //ros service
  ros_chassis_mode_srv_ = ros_nh_.advertiseService("set_chassis_mode", &Chassis::SetChassisModeService, this);

  //ros_message_init
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_link";

  odom_tf_.header.frame_id = "odom";
  odom_tf_.child_frame_id = "base_link";

  uwb_data_.header.frame_id = "uwb";
}
void Chassis::ChassisInfoCallback(const std::shared_ptr<roborts_sdk::p_chassis_info_t> chassis_info){

  ros::Time current_time = ros::Time::now();
  odom_.header.stamp = current_time;
  odom_.pose.pose.position.x = chassis_info->position_x_mm/1000.;
  odom_.pose.pose.position.y = chassis_info->position_y_mm/1000.;
  odom_.pose.pose.position.z = 0.0;
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(chassis_info->gyro_angle / 180.0 * M_PI);
  odom_.pose.pose.orientation = q;
  odom_.twist.twist.linear.x = chassis_info->v_x_mm / 1000.0;
  odom_.twist.twist.linear.y = chassis_info->v_y_mm / 1000.0;
  odom_.twist.twist.angular.z = chassis_info->gyro_palstance / 180.0 * M_PI;
  ros_odom_pub_.publish(odom_);

  odom_tf_.header.stamp = current_time;
  odom_tf_.transform.translation.x = chassis_info->position_x_mm/1000.;
  odom_tf_.transform.translation.y = chassis_info->position_y_mm/1000.;

  odom_tf_.transform.translation.z = 0.0;
  odom_tf_.transform.rotation = q;
  tf_broadcaster_.sendTransform(odom_tf_);

}
void Chassis::UWBInfoCallback(const std::shared_ptr<roborts_sdk::p_uwb_data_t> uwb_info){

  uwb_data_.header.stamp = ros::Time::now();
  uwb_data_.pose.position.x = ((double)uwb_info->x)/100.0;
  uwb_data_.pose.position.y = ((double)uwb_info->y)/100.0;
  uwb_data_.pose.position.z = 0;
  uwb_data_.pose.orientation = tf::createQuaternionMsgFromYaw(uwb_info->yaw);
  ros_uwb_pub_.publish(uwb_data_);

}
bool Chassis::SetChassisModeService(roborts_msgs::ChassisMode::Request  &req,
                                    roborts_msgs::ChassisMode::Response  &res){

  roborts_sdk::chassis_mode_e chassis_mode =
      static_cast<roborts_sdk::chassis_mode_e>(req.chassis_mode);
  chassis_mode_pub_->Publish(chassis_mode);

  res.received = true;
  return true;
}
void Chassis::ChassisSpeedCtrlCallback(const geometry_msgs::Twist::ConstPtr &vel){
  roborts_sdk::p_chassis_speed_t chassis_speed;
  chassis_speed.vx = vel->linear.x*1000;
  chassis_speed.vy = vel->linear.y*1000;
  chassis_speed.vw = vel->angular.z * 180.0 / M_PI;
  chassis_speed.rotate_x_offset = 0;
  chassis_speed.rotate_y_offset = 0;
  chassis_speed_pub_->Publish(chassis_speed);
}

void Chassis::ChassisSpeedAccCtrlCallback(const roborts_msgs::TwistAccel::ConstPtr &vel_acc){
  roborts_sdk::p_chassis_spd_acc_t chassis_spd_acc;
  chassis_spd_acc.vx = vel_acc->twist.linear.x*1000;
  chassis_spd_acc.vy = vel_acc->twist.linear.y*1000;
  chassis_spd_acc.vw = vel_acc->twist.angular.z * 180.0 / M_PI;
  chassis_spd_acc.ax = vel_acc->accel.linear.x*1000;
  chassis_spd_acc.ay = vel_acc->accel.linear.y*1000;
  chassis_spd_acc.wz = vel_acc->accel.angular.z * 180.0 / M_PI;
  chassis_spd_acc_pub_->Publish(chassis_spd_acc);
}
}
