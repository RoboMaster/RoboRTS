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

#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>

#include "roborts_msgs/GlobalPlannerAction.h"
#include "state/error_code.h"

using roborts_common::ErrorCode;

class GlobalPlannerTest{
 public:
  GlobalPlannerTest():
      global_planner_actionlib_client_("global_planner_node_action", true){
    ros::NodeHandle rviz_nh("move_base_simple");
    goal_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1,
                                                              &GlobalPlannerTest::GoalCallback,this);

    global_planner_actionlib_client_.waitForServer();
  }
  ~GlobalPlannerTest() = default;

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr & goal){
    ROS_INFO("Get new goal.");
    command_.goal = *goal;
    global_planner_actionlib_client_.sendGoal(command_,
                                              boost::bind(&GlobalPlannerTest::DoneCallback, this, _1, _2),
                                              boost::bind(&GlobalPlannerTest::ActiveCallback, this),
                                              boost::bind(&GlobalPlannerTest::FeedbackCallback, this, _1)
    );
  }

  void DoneCallback(const actionlib::SimpleClientGoalState& state,  const roborts_msgs::GlobalPlannerResultConstPtr& result){
    ROS_INFO("The goal is done with %s!",state.toString().c_str());
  }
  void ActiveCallback() {
    ROS_INFO("Action server has recived the goal, the goal is active!");
  }
  void FeedbackCallback(const roborts_msgs::GlobalPlannerFeedbackConstPtr& feedback){
    if (feedback->error_code != ErrorCode::OK) {
      ROS_INFO("%s", feedback->error_msg.c_str());
    }
    if (!feedback->path.poses.empty()) {
      ROS_INFO("Get Path!");
    }
  }
 private:
  ros::Subscriber goal_sub_;
  roborts_msgs::GlobalPlannerGoal command_;
  actionlib::SimpleActionClient<roborts_msgs::GlobalPlannerAction> global_planner_actionlib_client_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_planner_test");
  GlobalPlannerTest global_planner_test;
  ros::spin();
  return 0;
}

