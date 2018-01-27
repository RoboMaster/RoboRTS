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

#include "common/main_interface.h"
#include "modules/driver/serial/serial_comm_write.h"
#include "infantry_info.h"

namespace rrts {
namespace driver {
namespace serial {

SerialCommWrite::SerialCommWrite(std::string name): SerialComm(name) {
};

void SerialCommWrite::Init() {
  Initialization();
  ros::NodeHandle nh;
  gimbal_ctrl_subscriber_ = nh.subscribe("enemy_pos", 1, &SerialCommWrite::GimbalControlCallback, this);
  chassis_ctrl_subscriber_ = nh.subscribe("cmd_vel", 1, &SerialCommWrite::ChassisControlCallback, this);
}

void SerialCommWrite::GimbalControlCallback(const messages::EnemyPosConstPtr &msg) {
  gimbal_control_data_.ctrl_mode = GIMBAL_POSITION_MODE;
  gimbal_control_data_.pit_ref = msg->enemy_pitch;
  gimbal_control_data_.yaw_ref = msg->enemy_yaw;
  gimbal_control_data_.visual_valid = 1;
}

void SerialCommWrite::ChassisControlCallback(const geometry_msgs::Twist::ConstPtr &vel) {
  chassis_control_data_.ctrl_mode = AUTO_FOLLOW_GIMBAL;
  chassis_control_data_.x_speed = vel->linear.x * 1000.0;
  chassis_control_data_.y_speed = vel->linear.y * 1000.0;
  chassis_control_data_.w_info.x_offset = 0;
  chassis_control_data_.w_info.y_offset = 0;
  chassis_control_data_.w_info.w_speed = vel->angular.z;
}

void SerialCommWrite::Run() {
  int length = 0;
  if(gimbal_control_data_.visual_valid) {
    length = sizeof(gimbal_control_data_);
    SendDataHandle(GIMBAL_CTRL_ID, (uint8_t *) &gimbal_control_data_, length);
    if(SendData(length) != length) {
      LOG_WARNING<<"Gimbal control data sent error";
    }
  }
  length = sizeof(ChassisControl);
  SendDataHandle(CHASSIS_CTRL_ID, (uint8_t*) &chassis_control_data_, length);
  if(SendData(length) != length) {
    LOG_WARNING<<"Chassis control data sent error";
  }
}

void SerialCommWrite::Stop() {
  close(fd_);
}

void SerialCommWrite::SendDataHandle(uint16_t cmd_id, uint8_t *p_data, uint16_t len) {
  FrameHeader *p_header = (FrameHeader *) tx_buf_;
  p_header->sof = UP_REG_ID;
  p_header->data_length = len;
  memcpy(&tx_buf_[HEADER_LEN], (uint8_t *) &cmd_id, CMD_LEN);
  AppendCrcOctCheckSum(tx_buf_, HEADER_LEN);
  memcpy(&tx_buf_[HEADER_LEN + CMD_LEN], p_data, len);
  AppendCrcHexCheckSum(tx_buf_, HEADER_LEN + CMD_LEN + len + CRC_LEN);
}

int SerialCommWrite::SendData(int data_len) {
  int length = 0;
  length = write(fd_, tx_buf_, data_len);
  if (length == data_len) {
    return length;
  } else {
    tcflush(fd_, TCOFLUSH);
    LOG_FATAL << "Serial write error";
    return -1;
  }
}

}// namespace serial
}// namespace driver
}// namespace rrts
