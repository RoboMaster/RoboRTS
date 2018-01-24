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

#ifndef MODULES_DRIVER_SERIAL_SERIAL_COMM_READ_H
#define MODULES_DRIVER_SERIAL_SERIAL_COMM_READ_H

#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "modules/driver/serial/serial_comm.h"
#include "modules/driver/serial/infantry_info.h"
#include "modules/driver/serial/proto/serial_comm_config.pb.h"

namespace rrts {
namespace driver {
namespace serial {
class SerialCommRead : public SerialComm {
 public:
  SerialCommRead(std::string name);

  /**
   * @brief Get the module name
   * @return Return the class name.
   */
  std::string ModuleName() const;

  /**
   * @brief Serial port initialization.
   */
  void Init();

  /**
   * @brief Receive data from serial port and publish.
   */
  void Run();

  /**
   * @brief Close the serial port.
   */
  void Stop();

  //TODO(krik zhang): make accesses to the received data.
  void GetGameInfo(game_robot_state_t & game_info) {
    read_mutex_.lock();
    memcpy(&game_info, &game_information_, sizeof(game_robot_state_t));
    read_mutex_.unlock();
  }

 private:

  /**
   * @brief Receive data from the serial
   * @param fd File descriptor for serial port
   * @param rx_buf Received data buffer
   * @param data_len Maximum data length
   * @return The real length of received data.
   */
  int ReceiveDate(int fd, int data_len);

  ros::NodeHandle nh_;
  ros::Publisher odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
  tf::TransformBroadcaster tf_broadcaster_;
  unpack_step_e unpack_step_e_;
  uint8_t rx_buf_[UART_BUFF_SIZE];
  uint8_t byte_, protocol_packet_[PROTOCAL_FRAME_MAX_SIZE];
  uint16_t data_length_;
  int32_t read_len_, read_buff_index_, index_;
  bool stop_loop_, time_to_process_;
  void ComLoop();

  void data_handle();
  std::thread *receive_loop_;
  std::mutex read_mutex_;
  uint16_t computer_cmd_id_;
  frame_header_t computer_frame_header_;
  game_robot_state_t game_information_;
  robot_hurt_data_t robot_hurt_data_;
  real_shoot_data_t real_shoot_data_;
  rfid_detect_t rfid_data_;
  game_result_t game_result_data_;
  chassis_info_t chassis_information_;
  gimbal_info_t gimbal_information_;
  shoot_info_t shoot_task_data_;
  infantry_err_t global_error_data_;
  get_buff_t get_buff_data_;
  server_to_user_t student_download_data_;
  config_response_t config_response_data_;
  cali_response_t cali_response_data_;
  rc_info_t rc_info_data_;
  version_info_t version_info_data_;
};
} //namespace serial
} //namespace driver
} //namespace rrts
#endif //MODULES_DRIVER_SERIAL_SERIAL_COMM_READ_H
