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

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_ODOM_INFO_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_ODOM_INFO_H

#include <string>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


namespace roborts_local_planner {

/**
 * @brief A class of odometry
 */
class OdomInfo {
 public:

  /**
   * Constructor
   * @param topic Topic of odom info, default is null
   */
  OdomInfo(std::string topic = "");
  ~OdomInfo() {}

  /**
   * @brief Odom's callback function
   * @param msg Odom's messages
   */
  void OdomCB(const nav_msgs::Odometry::ConstPtr& msg);

  /**
   * @brief Get velocity
   * @param vel Velocity
   */
  void GetVel(tf::Stamped<tf::Pose>& vel);

  /**
   * @brief Set Odom callback
   * @param topic topic of odom info
   */
  void SetTopic(std::string topic);

  /**
   * @brief Get odom topic
   * @return Odom topic name
   */
  std::string GetTopic() const { return topic_; }

 private:

  //! odom topic name
  std::string topic_;
  //! subscriber
  ros::Subscriber sub_;
  //! odom info
  nav_msgs::Odometry odom_;
  //! odom mutex
  boost::mutex mutex_;
  //! odom frame id
  std::string frame_id_;

};

} // namespace roborts_local_planner
#endif //ROBORTS_PLANNING_LOCAL_PLANNER_ODOM_INFO_H
