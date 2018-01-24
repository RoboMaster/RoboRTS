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

#include "modules/planning/local_planner/local_visualization.h"

namespace rrts {
namespace planning {
namespace local_planner {
LocalVisualization::LocalVisualization() : initialized_(false){

}
LocalVisualization::LocalVisualization(ros::NodeHandle &nh, const std::string &visualize_frame) : initialized_(false){
  Initialization(nh, visualize_frame);
}
void LocalVisualization::Initialization(ros::NodeHandle &nh, const std::string &visualize_frame) {
  if (initialized_) {

  }

  visual_frame_ = visualize_frame;
  local_planner_ = nh.advertise<nav_msgs::Path>("local_planner", 1);
  pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("pose", 1);

  initialized_ = true;
}

void LocalVisualization::PublishLocalPlan(const TebVertexConsole& vertex_console) const{

  nav_msgs::Path local_plan;
  local_plan.header.frame_id = visual_frame_;
  local_plan.header.stamp = ros::Time::now();

  geometry_msgs::PoseArray local_pose;
  local_pose.header.frame_id = local_plan.header.frame_id;
  local_pose.header.stamp = local_plan.header.stamp;

  for (int i = 0; i <vertex_console.SizePoses(); ++i) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = local_plan.header.frame_id;
    pose_stamped.header.stamp = local_plan.header.stamp;
    pose_stamped.pose.position.x = vertex_console.Pose(i).GetPosition().coeffRef(0);
    pose_stamped.pose.position.y = vertex_console.Pose(i).GetPosition().coeffRef(1);
    pose_stamped.pose.position.z = 0;
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(vertex_console.Pose(i).GetTheta());
    local_plan.poses.push_back(pose_stamped);
    local_pose.poses.push_back(pose_stamped.pose);
  }
  local_planner_.publish(local_plan);
  pose_pub_.publish(local_pose);
}

} // namespace local_planner
} // namespace planning
} // namespace rrts

