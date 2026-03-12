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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "gimbal.h"
#include "../roborts_sdk/sdk.h"

#include <cmath>
#include <chrono>
#include <functional>

namespace roborts_base {

namespace {
  constexpr double kDeciDegreesPerRadian = 1800.0 / M_PI;
  constexpr double kRadiansPerDeciDegree = M_PI / 1800.0;
  constexpr int kHeartbeatIntervalMs = 300;
  constexpr double kGimbalHeight = 0.15;  // meters above base_link
  // Friction wheel motor PWM values
  constexpr uint16_t kFricWheelOpenSpeed  = 1240;
  constexpr uint16_t kFricWheelCloseSpeed = 1000;
  // Default shoot frequency
  constexpr uint16_t kDefaultShootFreq = 1500;
} // anonymous namespace

Gimbal::Gimbal(std::shared_ptr<roborts_sdk::Handle> handle)
    : handle_(std::move(handle)) {
  SDK_Init();
  ROS_Init();
}

Gimbal::~Gimbal() {
  if (heartbeat_thread_.joinable()) {
    heartbeat_thread_.join();
  }
}

void Gimbal::SDK_Init() {
  // Version query
  version_client_ = handle_->CreateClient<roborts_sdk::cmd_version_id, roborts_sdk::cmd_version_id>(
      UNIVERSAL_CMD_SET, CMD_REPORT_VERSION,
      MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);

  roborts_sdk::cmd_version_id version_cmd;
  version_cmd.version_id = 0;
  auto version = std::make_shared<roborts_sdk::cmd_version_id>(version_cmd);
  version_client_->AsyncSendRequest(version,
      [](roborts_sdk::Client<roborts_sdk::cmd_version_id,
                             roborts_sdk::cmd_version_id>::SharedFuture future) {
        const auto id = future.get()->version_id;
        ROS_INFO("Gimbal Firmware Version: %d.%d.%d.%d",
                 static_cast<int>((id >> 24) & 0xFF),
                 static_cast<int>((id >> 16) & 0xFF),
                 static_cast<int>((id >> 8) & 0xFF),
                 static_cast<int>(id & 0xFF));
      });

  // Gimbal info subscriber
  handle_->CreateSubscriber<roborts_sdk::cmd_gimbal_info>(
      GIMBAL_CMD_SET, CMD_PUSH_GIMBAL_INFO,
      GIMBAL_ADDRESS, BROADCAST_ADDRESS,
      std::bind(&Gimbal::GimbalInfoCallback, this, std::placeholders::_1));

  // SDK publishers
  gimbal_angle_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_gimbal_angle>(
      GIMBAL_CMD_SET, CMD_SET_GIMBAL_ANGLE,
      MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
  gimbal_mode_pub_ = handle_->CreatePublisher<roborts_sdk::gimbal_mode_e>(
      GIMBAL_CMD_SET, CMD_SET_GIMBAL_MODE,
      MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
  fric_wheel_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_fric_wheel_speed>(
      GIMBAL_CMD_SET, CMD_SET_FRIC_WHEEL_SPEED,
      MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
  gimbal_shoot_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_shoot_info>(
      GIMBAL_CMD_SET, CMD_SET_SHOOT_INFO,
      MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);

  // Heartbeat
  heartbeat_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_heartbeat>(
      UNIVERSAL_CMD_SET, CMD_HEARTBEAT,
      MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);

  heartbeat_thread_ = std::thread([this] {
    roborts_sdk::cmd_heartbeat heartbeat;
    heartbeat.heartbeat = 0;
    while (ros::ok()) {
      heartbeat_pub_->Publish(heartbeat);
      std::this_thread::sleep_for(std::chrono::milliseconds(kHeartbeatIntervalMs));
    }
  });
}

void Gimbal::ROS_Init() {
  // ROS subscriber
  ros_sub_cmd_gimbal_angle_ = ros_nh_.subscribe(
      "cmd_gimbal_angle", 1, &Gimbal::GimbalAngleCtrlCallback, this);

  // ROS services
  ros_gimbal_mode_srv_ = ros_nh_.advertiseService(
      "set_gimbal_mode", &Gimbal::SetGimbalModeService, this);
  ros_ctrl_fric_wheel_srv_ = ros_nh_.advertiseService(
      "cmd_fric_wheel", &Gimbal::CtrlFricWheelService, this);
  ros_ctrl_shoot_srv_ = ros_nh_.advertiseService(
      "cmd_shoot", &Gimbal::CtrlShootService, this);

  // Initialize TF message
  gimbal_tf_.header.frame_id = "base_link";
  gimbal_tf_.child_frame_id = "gimbal";
}

void Gimbal::GimbalInfoCallback(
    const std::shared_ptr<roborts_sdk::cmd_gimbal_info> gimbal_info) {
  const ros::Time current_time = ros::Time::now();

  const geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(
      0.0,
      gimbal_info->pitch_ecd_angle * kRadiansPerDeciDegree,
      gimbal_info->yaw_ecd_angle * kRadiansPerDeciDegree);

  gimbal_tf_.header.stamp = current_time;
  gimbal_tf_.transform.rotation = q;
  gimbal_tf_.transform.translation.x = 0;
  gimbal_tf_.transform.translation.y = 0;
  gimbal_tf_.transform.translation.z = kGimbalHeight;
  tf_broadcaster_.sendTransform(gimbal_tf_);
}

void Gimbal::GimbalAngleCtrlCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg) {
  roborts_sdk::cmd_gimbal_angle gimbal_angle;
  gimbal_angle.ctrl.bit.pitch_mode = msg->pitch_mode;
  gimbal_angle.ctrl.bit.yaw_mode = msg->yaw_mode;
  gimbal_angle.pitch = static_cast<int16_t>(msg->pitch_angle * kDeciDegreesPerRadian);
  gimbal_angle.yaw = static_cast<int16_t>(msg->yaw_angle * kDeciDegreesPerRadian);
  gimbal_angle_pub_->Publish(gimbal_angle);
}

bool Gimbal::SetGimbalModeService(roborts_msgs::GimbalMode::Request &req,
                                  roborts_msgs::GimbalMode::Response &res) {
  auto gimbal_mode = static_cast<roborts_sdk::gimbal_mode_e>(req.gimbal_mode);
  gimbal_mode_pub_->Publish(gimbal_mode);
  res.received = true;
  return true;
}

bool Gimbal::CtrlFricWheelService(roborts_msgs::FricWhl::Request &req,
                                  roborts_msgs::FricWhl::Response &res) {
  roborts_sdk::cmd_fric_wheel_speed fric_speed;
  if (req.open) {
    fric_speed.left = kFricWheelOpenSpeed;
    fric_speed.right = kFricWheelOpenSpeed;
  } else {
    fric_speed.left = kFricWheelCloseSpeed;
    fric_speed.right = kFricWheelCloseSpeed;
  }
  fric_wheel_pub_->Publish(fric_speed);
  res.received = true;
  return true;
}

bool Gimbal::CtrlShootService(roborts_msgs::ShootCmd::Request &req,
                              roborts_msgs::ShootCmd::Response &res) {
  roborts_sdk::cmd_shoot_info gimbal_shoot;

  switch (static_cast<roborts_sdk::shoot_cmd_e>(req.mode)) {
    case roborts_sdk::SHOOT_STOP:
      gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_STOP;
      gimbal_shoot.shoot_add_num = 0;
      gimbal_shoot.shoot_freq = 0;
      break;

    case roborts_sdk::SHOOT_ONCE:
      gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_ONCE;
      gimbal_shoot.shoot_add_num = (req.number != 0) ? req.number : 1;
      gimbal_shoot.shoot_freq = kDefaultShootFreq;
      break;

    case roborts_sdk::SHOOT_CONTINUOUS:
      gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_CONTINUOUS;
      gimbal_shoot.shoot_add_num = req.number;
      gimbal_shoot.shoot_freq = kDefaultShootFreq;
      break;

    default:
      ROS_WARN("Unknown shoot mode: %d", req.mode);
      return false;
  }

  gimbal_shoot_pub_->Publish(gimbal_shoot);
  res.received = true;
  return true;
}

} // namespace roborts_base
