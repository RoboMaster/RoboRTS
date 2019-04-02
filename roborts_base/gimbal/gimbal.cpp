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

#include "gimbal.h"
#include "../roborts_sdk/sdk.h"

namespace roborts_base{
Gimbal::Gimbal(std::shared_ptr<roborts_sdk::Handle> handle):
    handle_(handle){
  SDK_Init();
  ROS_Init();
}

Gimbal::~Gimbal(){
  if(heartbeat_thread_.joinable()){
    heartbeat_thread_.join();
  }
}

void Gimbal::SDK_Init(){

  verison_client_ = handle_->CreateClient<roborts_sdk::cmd_version_id,roborts_sdk::cmd_version_id>
      (UNIVERSAL_CMD_SET, CMD_REPORT_VERSION,
       MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
  roborts_sdk::cmd_version_id version_cmd;
  version_cmd.version_id=0;
  auto version = std::make_shared<roborts_sdk::cmd_version_id>(version_cmd);
  verison_client_->AsyncSendRequest(version,
                                    [](roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                                           roborts_sdk::cmd_version_id>::SharedFuture future) {
                                      ROS_INFO("Gimbal Firmware Version: %d.%d.%d.%d",
                                               int(future.get()->version_id>>24&0xFF),
                                               int(future.get()->version_id>>16&0xFF),
                                               int(future.get()->version_id>>8&0xFF),
                                               int(future.get()->version_id&0xFF));
                                    });

  handle_->CreateSubscriber<roborts_sdk::cmd_gimbal_info>(GIMBAL_CMD_SET, CMD_PUSH_GIMBAL_INFO,
                                                          GIMBAL_ADDRESS, BROADCAST_ADDRESS,
                                                          std::bind(&Gimbal::GimbalInfoCallback, this, std::placeholders::_1));

  gimbal_angle_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_gimbal_angle>(GIMBAL_CMD_SET, CMD_SET_GIMBAL_ANGLE,
                                                                              MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
  gimbal_mode_pub_ = handle_->CreatePublisher<roborts_sdk::gimbal_mode_e>(GIMBAL_CMD_SET, CMD_SET_GIMBAL_MODE,
                                                                          MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
  fric_wheel_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_fric_wheel_speed>(GIMBAL_CMD_SET, CMD_SET_FRIC_WHEEL_SPEED,
                                                                                MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
  gimbal_shoot_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_shoot_info>(GIMBAL_CMD_SET, CMD_SET_SHOOT_INFO,
                                                                            MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);

  heartbeat_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_heartbeat>(UNIVERSAL_CMD_SET, CMD_HEARTBEAT,
                                                                        MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
  heartbeat_thread_ = std::thread([this]{
                                    roborts_sdk::cmd_heartbeat heartbeat;
                                    heartbeat.heartbeat=0;
                                    while(ros::ok()){
                                      heartbeat_pub_->Publish(heartbeat);
                                      std::this_thread::sleep_for(std::chrono::milliseconds(300));
                                    }
                                  }
  );
}

void Gimbal::ROS_Init(){

  //ros subscriber
  ros_sub_cmd_gimbal_angle_ = ros_nh_.subscribe("cmd_gimbal_angle", 1, &Gimbal::GimbalAngleCtrlCallback, this);

  //ros service
  ros_gimbal_mode_srv_ = ros_nh_.advertiseService("set_gimbal_mode", &Gimbal::SetGimbalModeService, this);
  ros_ctrl_fric_wheel_srv_ = ros_nh_.advertiseService("cmd_fric_wheel", &Gimbal::CtrlFricWheelService, this);
  ros_ctrl_shoot_srv_ = ros_nh_.advertiseService("cmd_shoot", &Gimbal::CtrlShootService, this);
  //ros_message_init
  gimbal_tf_.header.frame_id = "base_link";
  gimbal_tf_.child_frame_id = "gimbal";

}

void Gimbal::GimbalInfoCallback(const std::shared_ptr<roborts_sdk::cmd_gimbal_info> gimbal_info){

  ros::Time current_time = ros::Time::now();
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0,
                                                                        gimbal_info->pitch_ecd_angle / 1800.0 * M_PI,
                                                                        gimbal_info->yaw_ecd_angle / 1800.0 * M_PI);
  gimbal_tf_.header.stamp = current_time;
  gimbal_tf_.transform.rotation = q;
  gimbal_tf_.transform.translation.x = 0;
  gimbal_tf_.transform.translation.y = 0;
  gimbal_tf_.transform.translation.z = 0.15;
  tf_broadcaster_.sendTransform(gimbal_tf_);

}

void Gimbal::GimbalAngleCtrlCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg){

  roborts_sdk::cmd_gimbal_angle gimbal_angle;
  gimbal_angle.ctrl.bit.pitch_mode = msg->pitch_mode;
  gimbal_angle.ctrl.bit.yaw_mode = msg->yaw_mode;
  gimbal_angle.pitch = msg->pitch_angle*1800/M_PI;
  gimbal_angle.yaw = msg->yaw_angle*1800/M_PI;

  gimbal_angle_pub_->Publish(gimbal_angle);

}

bool Gimbal::SetGimbalModeService(roborts_msgs::GimbalMode::Request &req,
                                  roborts_msgs::GimbalMode::Response &res){
  roborts_sdk::gimbal_mode_e gimbal_mode = static_cast<roborts_sdk::gimbal_mode_e>(req.gimbal_mode);
  gimbal_mode_pub_->Publish(gimbal_mode);
  res.received = true;
  return true;
}
bool Gimbal::CtrlFricWheelService(roborts_msgs::FricWhl::Request &req,
                                  roborts_msgs::FricWhl::Response &res){
  roborts_sdk::cmd_fric_wheel_speed fric_speed;
  if(req.open){
    fric_speed.left = 1240;
    fric_speed.right = 1240;
  } else{
    fric_speed.left = 1000;
    fric_speed.right = 1000;
  }
  fric_wheel_pub_->Publish(fric_speed);
  res.received = true;
  return true;
}
bool Gimbal::CtrlShootService(roborts_msgs::ShootCmd::Request &req,
                              roborts_msgs::ShootCmd::Response &res){
  roborts_sdk::cmd_shoot_info gimbal_shoot;
  uint16_t default_freq = 1500;
  switch(static_cast<roborts_sdk::shoot_cmd_e>(req.mode)){
    case roborts_sdk::SHOOT_STOP:
      gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_STOP;
      gimbal_shoot.shoot_add_num = 0;
      gimbal_shoot.shoot_freq = 0;
      break;
    case roborts_sdk::SHOOT_ONCE:
      if(req.number!=0){
        gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_ONCE;
        gimbal_shoot.shoot_add_num = req.number;
        gimbal_shoot.shoot_freq = default_freq;
      }
      else{
        gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_ONCE;
        gimbal_shoot.shoot_add_num = 1;
        gimbal_shoot.shoot_freq = default_freq;
      }
      break;
    case roborts_sdk::SHOOT_CONTINUOUS:
      gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_CONTINUOUS;
      gimbal_shoot.shoot_add_num = req.number;
      gimbal_shoot.shoot_freq = default_freq;
      break;
    default:
      return  false;
  }
  gimbal_shoot_pub_->Publish(gimbal_shoot);

  res.received = true;
  return true;
}
}
