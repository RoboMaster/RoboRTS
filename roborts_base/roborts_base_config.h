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

#ifndef ROBORTS_BASE_CONFIG_H
#define ROBORTS_BASE_CONFIG_H
#include <ros/ros.h>

namespace roborts_base{

struct Config {
  void GetParam(ros::NodeHandle *nh) {
    nh->param<std::string>("serial_port", serial_port, "/dev/serial_sdk");
  }
  std::string serial_port;
};

}
#endif //ROBORTS_BASE_CONFIG_H