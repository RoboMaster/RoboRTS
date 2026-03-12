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

#ifndef ROBORTS_BASE_CONFIG_H
#define ROBORTS_BASE_CONFIG_H

#include <string>
#include <ros/ros.h>

namespace roborts_base {

/**
 * @brief Configuration struct for roborts_base node.
 *
 * Loads parameters from the ROS parameter server with sensible defaults.
 * Supports serial port configuration, loop rate, and debug mode toggling.
 */
struct Config {
  /// @brief Load all parameters from the ROS parameter server.
  /// @param nh Pointer to a ROS NodeHandle (typically private ~)
  void GetParam(ros::NodeHandle *nh) {
    nh->param<std::string>("serial_port", serial_port, "/dev/serial_sdk");
    nh->param<int>("loop_rate", loop_rate, 1000);
    nh->param<bool>("enable_chassis", enable_chassis, true);
    nh->param<bool>("enable_gimbal", enable_gimbal, true);
    nh->param<bool>("enable_referee", enable_referee, true);
  }

  //! Serial port device path
  std::string serial_port;
  //! Main loop rate in Hz (default: 1000 Hz)
  int loop_rate = 1000;
  //! Enable/disable individual hardware modules
  bool enable_chassis = true;
  bool enable_gimbal = true;
  bool enable_referee = true;
};

} // namespace roborts_base

#endif // ROBORTS_BASE_CONFIG_H