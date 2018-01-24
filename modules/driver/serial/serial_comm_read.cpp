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

#include <boost/bind.hpp>
#include "common/main_interface.h"
#include "modules/driver/serial/serial_comm_read.h"

namespace rrts {
namespace driver {
namespace serial {

SerialCommRead::SerialCommRead(std::string name) : SerialComm(name),
                                                   stop_loop_(false),
                                                   time_to_process_(false),
                                                   read_buff_index_(0),
                                                   unpack_step_e_(STEP_HEADER_SOF),
                                                   index_(0),
                                                   byte_(0),
                                                   data_length_(0) {
}

std::string SerialCommRead::ModuleName() const {
  return std::string("SerialCommRead");
}

void SerialCommRead::Init() {
  Initialization();
  memset(rx_buf_, 0, UART_BUFF_SIZE);
}

void SerialCommRead::Run() {
  receive_loop_ = new std::thread(boost::bind(&SerialCommRead::ComLoop, this));
}

void SerialCommRead::ComLoop() {
  while (!stop_loop_) {
    read_buff_index_ = 0;
    read_len_ = ReceiveDate(fd_, UART_BUFF_SIZE);
    while (read_len_--) {
      byte_ = rx_buf_[read_buff_index_++];
      switch (unpack_step_e_) {
        case STEP_HEADER_SOF: {
          if (byte_ == UP_REG_ID) {
            protocol_packet_[index_++] = byte_;
            unpack_step_e_ = STEP_LENGTH_LOW;
          } else {
            index_ = 0;
          }
        }
          break;
        case STEP_LENGTH_LOW: {
          data_length_ = byte_;
          protocol_packet_[index_++] = byte_;
          unpack_step_e_ = STEP_LENGTH_HIGH;
        }
          break;
        case STEP_LENGTH_HIGH: {
          data_length_ |= (byte_ << 8);
          protocol_packet_[index_++] = byte_;
          if (data_length_ < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CMD_LEN - CRC_LEN)) {
            unpack_step_e_ = STEP_FRAME_SEQ;
          } else {
            unpack_step_e_ = STEP_HEADER_SOF;
            index_ = 0;
          }
        }
          break;
        case STEP_FRAME_SEQ: {
          protocol_packet_[index_++] = byte_;
          unpack_step_e_ = STEP_HEADER_CRC8;
        }
          break;
        case STEP_HEADER_CRC8: {
          protocol_packet_[index_++] = byte_;
          if ((index_ == HEADER_LEN) && Verify_crc8_check_sum(protocol_packet_, HEADER_LEN)) {
            unpack_step_e_ = STEP_DATA_CRC16;
          } else {
            unpack_step_e_ = STEP_HEADER_SOF;
            index_ = 0;
          }
        }
          break;
        case STEP_DATA_CRC16: {
          if (index_ < (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
            protocol_packet_[index_++] = byte_;
          } else if (index_ > (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
            LOG_WARNING << "Index Beyond";
          }
          if (index_ == (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
            unpack_step_e_ = STEP_HEADER_SOF;
            index_ = 0;
            if (Verify_crc16_check_sum(protocol_packet_, HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
              data_handle();
            } else {
              LOG_WARNING<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>CRC16 INVALID<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<";
            }
          }
        }
          break;
        default: {
          LOG_WARNING<<"Unpack not well";
          unpack_step_e_ = STEP_HEADER_SOF;
          index_ = 0;
        }
          break;
      }
    }
  }
}
void SerialCommRead::data_handle() {
  std::lock_guard<std::mutex> guard(read_mutex_);
  auto *p_header = (frame_header_t *) protocol_packet_;
  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id = *(uint16_t *) (protocol_packet_ + HEADER_LEN);
  uint8_t *data_addr = protocol_packet_ + HEADER_LEN + CMD_LEN;
  switch (cmd_id) {
    case GAME_INFO_ID: memcpy(&game_information_, data_addr, data_length);
      //use it to control pipeline.
      break;
    case REAL_BLOOD_DATA_ID: memcpy(&robot_hurt_data_, data_addr, data_length);
      break;
    case REAL_SHOOT_DATA_ID: memcpy(&real_shoot_data_, data_addr, data_length);
      break;
    case REAL_RFID_DATA_ID: memcpy(&rfid_data_, data_addr, data_length);
      break;
    case GAME_RESULT_ID: memcpy(&game_result_data_, data_addr, data_length);
      break;
    case GAIN_BUFF_ID: memcpy(&get_buff_data_, data_addr, data_length);
      break;
    case SERVER_TO_USER_ID: memcpy(&student_download_data_, data_addr, data_length);
      break;
    case CHASSIS_DATA_ID: {
      ros::Time current_time = ros::Time::now();
      memcpy(&chassis_information_, data_addr, data_length);
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      double x = chassis_information_.x_position / 1000, y = chassis_information_.y_position / 1000;
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(
          chassis_information_.gyro_angle * 0.5 + chassis_information_.ecd_calc_angle * 0.5);
      odom.pose.pose.orientation = q;
      odom.twist.twist.linear.x = (double) chassis_information_.x_speed / 1000;
      odom.twist.twist.linear.y = (double) chassis_information_.y_speed / 1000;
      odom.twist.twist.angular.z =
          (double) (chassis_information_.gyro_palstance + chassis_information_.ecd_palstance) / 2;
      odom_pub_.publish(odom);
      geometry_msgs::TransformStamped odom_tf;
      odom_tf.header.frame_id = "odom";
      odom_tf.header.stamp = current_time;
      odom_tf.child_frame_id = "base_link";
      odom_tf.transform.translation.x = x;
      odom_tf.transform.translation.y = y;
      odom_tf.transform.translation.z = 0.0;
      odom_tf.transform.rotation = q;
      tf_broadcaster_.sendTransform(odom_tf);
    }
      break;

    case GIMBAL_DATA_ID: memcpy(&gimbal_information_, data_addr, data_length);
      printf("-->%d\n", gimbal_information_.ctrl_mode);
      break;

    case SHOOT_TASK_DATA_ID: memcpy(&shoot_task_data_, data_addr, data_length);
      printf("-->%d\n", shoot_task_data_.fric_wheel_run);
      break;

    case INFANTRY_ERR_ID: memcpy(&global_error_data_, data_addr, data_length);
      printf("-->%d\n", global_error_data_.err_sta);
      break;

    case CONFIG_RESPONSE_ID: memcpy(&config_response_data_, data_addr, data_length);
      printf("-->%d\n", config_response_data_.chassis_config);
      break;

    case CALI_RESPONSE_ID: memcpy(&cali_response_data_, data_addr, data_length);
      printf("-->%d\n", cali_response_data_.type);
      break;

    case REMOTE_CTRL_INFO_ID: memcpy(&rc_info_data_, data_addr, data_length);
      printf("-->%d\n", rc_info_data_.ch1);
      break;

    case BOTTOM_VERSION_ID: memcpy(&version_info_data_, data_addr, data_length);
      printf("-->%d\n", version_info_data_.num[0]);
      break;

    default:LOG_WARNING << "Cannot Get Command ID";
      break;
  }
}

void SerialCommRead::Stop() {
  receive_loop_->join();
  delete receive_loop_;
  close(fd_);
}

int SerialCommRead::ReceiveDate(int fd, int data_len) {
  int len_received, fs_sel;
  fd_set fs_read;
  struct timeval time;
  FD_ZERO(&fs_read);
  FD_SET(fd, &fs_read);
  time.tv_sec = 10;
  time.tv_usec = 0;
  fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
  if (fs_sel > 0) {
    len_received = read(fd, rx_buf_, data_len);
  } else if (fs_sel == 0) {
    LOG_WARNING_EVERY(10000) << "Uart Timeout";
  } else {
    LOG_ERROR << "Select function error";
  }
  return len_received;
}

} // namespace serial
} // namespace driver
} // namespace modules

