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
  void GetGameInfo(GameInfo & game_info) {
    read_mutex_.lock();
    memcpy(&game_info, &game_information_, sizeof(GameInfo));
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
  int ReceiveDate(int fd, int data_length);
  void CommunicateLoop();
  void data_handle();

  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  UnpackStep unpack_step_e_;
  uint8_t rx_buf_[UART_BUFF_SIZE];
  uint8_t byte_, protocol_packet_[PROTOCAL_FRAME_MAX_SIZE];
  uint16_t data_length_;
  int32_t read_len_, read_buff_index_, index_;
  bool stop_loop_, time_to_process_;

  std::thread *receive_loop_;
  std::mutex read_mutex_;
  uint16_t computer_cmd_id_;
  FrameHeader computer_frame_header_;
  GameInfo game_information_;
  HurtData robot_hurt_data_;
  ShootData real_shoot_data_;
  RfidData rfid_data_;
  GameResult game_result_data_;
  ChassisInfo chassis_information_;
  GimbalInfo gimbal_information_;
  ShootInfo shoot_task_data_;
  InfantryError global_error_data_;
  GameBuff get_buff_data_;
  ServerToUser student_download_data_;
  ConfigMessage config_response_data_;
  CalibrateResponse cali_response_data_;
  RcInfo rc_info_data_;
  VersionInfo version_info_data_;
};
} //namespace serial
} //namespace driver
} //namespace rrts
#endif //MODULES_DRIVER_SERIAL_SERIAL_COMM_READ_H
