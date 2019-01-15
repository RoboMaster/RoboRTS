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

#include <mutex>
#include <thread>
#include <chrono>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "roborts_msgs/TwistAccel.h"


class VelConverter {
 public:
  VelConverter() : new_cmd_acc_(false), begin_(false) {
    cmd_vel_.linear.x = 0;
    cmd_vel_.linear.y = 0;
    cmd_vel_.angular.z = 0;

    cmd_pub_ = cmd_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    cmd_sub_ = cmd_handle_.subscribe<roborts_msgs::TwistAccel>("/cmd_vel_acc", 100, boost::bind(&VelConverter::VelCallback, this, _1));
  }
  void VelCallback(const roborts_msgs::TwistAccel::ConstPtr& msg);
  void UpdateVel();

 private:
  roborts_msgs::TwistAccel cmd_vel_acc_;
  geometry_msgs::Twist cmd_vel_;

  bool new_cmd_acc_, begin_;

  ros::NodeHandle cmd_handle_;
  ros::Publisher cmd_pub_;
  ros::Subscriber cmd_sub_;
  std::chrono::high_resolution_clock::time_point time_begin_;

  std::mutex cmd_mutex_;
};

void VelConverter::VelCallback(const roborts_msgs::TwistAccel::ConstPtr& twist_acc_msg)
{
  if (!begin_) {
    begin_ = true;
  }
  std::lock_guard<std::mutex> cmd_guard(cmd_mutex_);
  new_cmd_acc_ = true;
  cmd_vel_acc_ = *twist_acc_msg;
}

void VelConverter::UpdateVel() {
  if (!begin_) {
    return;
  }
  auto begin = std::chrono::high_resolution_clock::now();
  if (new_cmd_acc_) {
    std::lock_guard<std::mutex> cmd_guard(cmd_mutex_);
    cmd_vel_ = cmd_vel_acc_.twist;
    cmd_pub_.publish(cmd_vel_);
    new_cmd_acc_ = false;
    time_begin_ = std::chrono::high_resolution_clock::now();
    return;
  }
  auto actual_time = std::chrono::duration<double, std::ratio<1, 1>>(std::chrono::high_resolution_clock::now() - time_begin_).count();
  time_begin_ = std::chrono::high_resolution_clock::now();

  cmd_vel_.linear.x = cmd_vel_.linear.x + actual_time * cmd_vel_acc_.accel.linear.x;
  cmd_vel_.linear.y = cmd_vel_.linear.y + actual_time * cmd_vel_acc_.accel.linear.y;
  cmd_vel_.angular.z = cmd_vel_.angular.z + actual_time * cmd_vel_acc_.accel.angular.z;

  cmd_pub_.publish(cmd_vel_);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vel_converter");

  VelConverter vel_converter;

  while	(ros::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ros::spinOnce();
    vel_converter.UpdateVel();
  }

  return 0;
}