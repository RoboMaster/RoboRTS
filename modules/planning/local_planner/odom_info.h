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

#ifndef MODULES_PLANNING_LOCAL_PLANNER_ODOM_INFO_H
#define MODULES_PLANNING_LOCAL_PLANNER_ODOM_INFO_H

#include <string>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>

#include "messages/Odometry.h"

namespace rrts {
namespace planning {
namespace local_planner {

class OdomInfo {
 public:

  OdomInfo(std::string topic = "");
  ~OdomInfo() {}

  void OdomCB(const messages::Odometry::ConstPtr& msg);

  void GetVel(tf::Stamped<tf::Pose>& vel);

  void SetTopic(std::string topic);

  std::string GetTopic() const { return topic_; }

 private:

  std::string topic_;
  ros::Subscriber sub_;
  messages::Odometry odom_;
  boost::mutex mutex_;
  std::string frame_id_;

};

} // namespace local_planner
} // namespace planning
} // namespace rrts
#endif //MODULES_PLANNING_LOCAL_PLANNER_ODOM_INFO_H
