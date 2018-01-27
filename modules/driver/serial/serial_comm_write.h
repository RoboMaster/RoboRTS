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

#ifndef AUTO_PILOT_SERIAL_COMM_SERIAL_COMM_WRITE_H
#define AUTO_PILOT_SERIAL_COMM_SERIAL_COMM_WRITE_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "messages/EnemyPos.h"
#include "modules/driver/serial/serial_comm.h"
#include "modules/driver/serial/infantry_info.h"
#include "modules/driver/serial/proto/serial_comm_config.pb.h"

namespace rrts {
namespace driver {
namespace serial {

class SerialCommWrite : public SerialComm {
 public:
  SerialCommWrite(std::string name);

  /**
   * Serial write port initialization.
   */
  void Init();

  /**
   * @brief Start sending data to serial.
   */
  void Run();

  /**
   * @brief Close the serial port.
   */
  void Stop();

 private:

  /**
   *@brief Pack the message to send
   */
  void SendDataHandle(uint16_t cmd_id, uint8_t *p_data, uint16_t len);

  /**
   * @brief Send data through serial
   * @param data_len Data length to send.
   * @return data_len if sent ok, otherwise -1.
   */
  int SendData(int data_len);

 private:
  ros::Subscriber gimbal_ctrl_subscriber_;
  ros::Subscriber chassis_ctrl_subscriber_;
  ros::Subscriber shoot_ctrl_subcriber_;
  ros::Subscriber global_err_level_subscriber_;
  ros::Subscriber infantry_structure_subscriber_;
  ros::Subscriber cali_command_subscriber_;
  void GimbalControlCallback(const messages::EnemyPosConstPtr &msg);
  void ChassisControlCallback(const geometry_msgs::Twist::ConstPtr &vel);
  void ShootControlCallback();
  //TODO(krik zhang): make callbacks to receive the shoot control topic, infantry_structure, cali_command, etc.
  uint8_t tx_buf_[UART_BUFF_SIZE];
  GimbalControl gimbal_control_data_;
  ChassisControl chassis_control_data_;
  ShootControl shoot_contorl_data_;
  GlobalErrorLevel global_error_data_;
  InfantryStructure infantry_structure_data_;
  CalibrateCommand cali_command_data_;
};

}// namespace serial
}// namespace driver
}// namespace rrts

#endif //MODULES_DRIVER_SERIAL_SERIAL_COMM_WRITE_H
