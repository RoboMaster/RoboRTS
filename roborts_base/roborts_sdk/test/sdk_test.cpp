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

#include <iostream>

#include "../sdk.h"
#include "../protocol/protocol_define.h"

int main() {
  auto h=std::make_shared<Handle>();
  int count;

  /*-----------Subscriber Test-------------*/
  auto func = [&count] (const std::shared_ptr<p_chassis_info_t> message) -> void{
    DLOG_INFO<<"chassis_msg_"<<count<<" : "<<(int)(message->position_x_mm);
    count++;
  };
  auto func2 = [&count] (const std::shared_ptr<p_uwb_data_t> message) -> void{
    DLOG_INFO<<"chassis_msg_"<<count<<" : "<<(int)message->error;
    count++;
  };
  auto func3 = [&count] (const std::shared_ptr<p_gimbal_info_t> message) -> void{
    DLOG_INFO<<"chassis_msg_"<<count<<" : "<<(float)message->pit_relative_angle;
    count++;
  };
  auto sub1=h->CreateSubscriber<p_chassis_info_t>(CHASSIS_CMD_SET,PUSH_CHASSIS_INFO,CHASSIS_ADDRESS,MANIFOLD2_ADDRESS,func);
  auto sub2=h->CreateSubscriber<p_uwb_data_t>(CHASSIS_CMD_SET,PUSH_UWB_INFO,CHASSIS_ADDRESS,MANIFOLD2_ADDRESS,func2);
  auto sub3=h->CreateSubscriber<p_gimbal_info_t>(GIMBAL_CMD_SET,PUSH_GIMBAL_INFO,GIMBAL_ADDRESS,BROADCAST_ADDRESS,func3);

  /*-----------Publisher Test-------------*/
  auto pub1 = h->CreatePublisher<p_chassis_speed_t>(CHASSIS_CMD_SET,CTRL_CHASSIS_SPEED,MANIFOLD2_ADDRESS,CHASSIS_ADDRESS);
  p_chassis_speed_t chassis_speed;
  chassis_speed.rotate_x_offset=0;
  chassis_speed.rotate_y_offset=0;
  chassis_speed.vx=100;
  chassis_speed.vy=0;
  chassis_speed.vw=0;
  chassis_speed.res=0;
  pub1->Publish(chassis_speed);

   /*-----------Client Test-------------*/
  auto client1=h->CreateClient<chassis_mode_e,chassis_mode_e>(CHASSIS_CMD_SET,SET_CHASSIS_MODE,MANIFOLD2_ADDRESS,CHASSIS_ADDRESS);
  auto mode = std::make_shared<chassis_mode_e>(SEPARATE_GIMBAL);

  client1->AsyncSendRequest(mode,[](Client<chassis_mode_e,chassis_mode_e>::SharedFuture){
    std::cout<<"get!"<<std::endl;
  });

  while(true){
    h->Spin();
    usleep(10);
  }


}