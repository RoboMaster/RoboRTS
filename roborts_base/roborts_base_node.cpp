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

#include "gimbal/gimbal.h"
#include "chassis/chassis.h"
#include "referee_system/referee_system.h"
#include "roborts_base_config.h"


int main(int argc, char **argv){
  ros::init(argc, argv, "roborts_base_node");
  ros::NodeHandle nh;
  roborts_base::Config config;
  config.GetParam(&nh);
  auto handle = std::make_shared<roborts_sdk::Handle>(config.serial_port);
  if(!handle->Init()) return 1;

  roborts_base::Chassis chassis(handle);
  roborts_base::Gimbal gimbal(handle);
  roborts_base::RefereeSystem referee_system(handle);
  while(ros::ok()) {

    handle->Spin();
    ros::spinOnce();
    usleep(1000);
  }

}