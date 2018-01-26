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

#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <thread>
#include "messages/LocalPlannerAction.h"

#include "common/log.h"

nav_msgs::Path plan_info;
bool new_goal_flag = false;
void GoalCB (const geometry_msgs::PoseStamped::ConstPtr & goal);
void action_client ();

int main(int argc, char **argv) {
  ros::init(argc, argv, "local_test_client");

  ros::NodeHandle goal_nh;
  ros::Subscriber sub_goal = goal_nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, GoalCB);

  auto thread = std::thread(action_client);
  ros::spin();
  return 0;
}

void GoalCB (const geometry_msgs::PoseStamped::ConstPtr & goal) {
  if (!plan_info.poses.empty()) {
    plan_info.poses.clear();
  }
  plan_info.poses.push_back(*goal);
  new_goal_flag = true;
}

void action_client () {
  actionlib::SimpleActionClient<messages::LocalPlannerAction> ac("local_planner_node_action", true);
  LOG_INFO<<"Waiting for action server to start.";
  ac.waitForServer();
  LOG_INFO<<"Start.";
  messages::LocalPlannerGoal goal;

  char command = '0';

  while (ros::ok()) {
    if (new_goal_flag) {
      goal.route = plan_info;
      ac.sendGoal(goal);
      new_goal_flag = false;
    }
  }
}
