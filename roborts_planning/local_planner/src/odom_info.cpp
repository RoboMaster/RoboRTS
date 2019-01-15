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

#include "local_planner/odom_info.h"

namespace roborts_local_planner {
OdomInfo::OdomInfo(std::string topic) {
  SetTopic(topic);
}

void OdomInfo::OdomCB(const nav_msgs::Odometry::ConstPtr& msg) {

  boost::mutex::scoped_lock lock(mutex_);
  odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
  odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
  odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
  odom_.child_frame_id = msg->child_frame_id;

}

void OdomInfo::GetVel(tf::Stamped<tf::Pose>& vel) {
  geometry_msgs::Twist temp_vel;{
    boost::mutex::scoped_lock lock(mutex_);
    temp_vel.linear.x = odom_.twist.twist.linear.x;
    temp_vel.linear.y = odom_.twist.twist.linear.y;
    temp_vel.angular.z = odom_.twist.twist.angular.z;

    vel.frame_id_ = odom_.child_frame_id;
  }
  vel.setData(tf::Transform(tf::createQuaternionFromYaw(temp_vel.angular.z),
                                  tf::Vector3(temp_vel.linear.x, temp_vel.linear.y, 0)));
  vel.stamp_ = ros::Time();
}

void OdomInfo::SetTopic(std::string topic)
{
  if (topic_ == topic) {
    return;
  } else {
    topic_ = topic;
    if (topic == "") {
      sub_.shutdown();
      return;
    } else {
      ros::NodeHandle nh;
      sub_ = nh.subscribe<nav_msgs::Odometry>(topic_, 1, boost::bind( &OdomInfo::OdomCB, this, _1 ));
    }
  }
}

} // namespace roborts_local_planner