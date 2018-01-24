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

#include <std_msgs/String.h>
#include "modules/driver/serial/serial_comm_read.h"

int main(int argc,char* argv[]){

  ros::init(argc, argv, "serial_read_node");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("bottom_info", 1000);
  ros::Rate loop_rate(10);

  google::InitGoogleLogging("serial_read_node");
  google::SetStderrLogging(google::INFO);

  rrts::driver::serial::SerialCommRead serialCommRead("COM");
  serialCommRead.Init();
  serialCommRead.Run();
  while(1) {
    usleep(1000000);
    printf("Main Loop");
  }
//  while(ros::ok()){
//	ros::spinOnce();
//	loop_rate.sleep();
//  }
//  serialCommRead.Stop();
  return 0;
}
