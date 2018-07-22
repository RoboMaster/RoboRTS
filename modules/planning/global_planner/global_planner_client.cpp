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
#include <actionlib/client/simple_action_client.h>

#include "messages/GlobalPlannerAction.h"
#include "common/error_code.h"
#include "common/log.h"

using rrts::common::ErrorCode;
using rrts::common::ErrorInfo;

class GlobalPlannerClient{
 public:
  GlobalPlannerClient():global_planner_actionlib_client_("global_planner_node_action", true),
                        new_goal_(false){
    ros::NodeHandle rviz_nh("move_base_simple");
    goal_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1,
                                                              &GlobalPlannerClient::GoalCallback,this);
    global_planner_actionlib_client_.waitForServer();


    thread_ = std::thread(&GlobalPlannerClient::Execution,this);

  }
  ~GlobalPlannerClient(){
    if(thread_.joinable()){
      thread_.join();
    }
  }
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr & goal){
    command_.goal = *goal;
    new_goal_ = true;
  }
  void Execution(){
    while(ros::ok()) {
      if (new_goal_) {
        global_planner_actionlib_client_.sendGoal(command_,
                     boost::bind(&GlobalPlannerClient::DoneCallback, this, _1, _2),
                     boost::bind(&GlobalPlannerClient::ActiveCallback, this),
                     boost::bind(&GlobalPlannerClient::FeedbackCallback, this, _1)
        );
        new_goal_ = false;
      }
    }

  }
  void DoneCallback(const actionlib::SimpleClientGoalState& state,  const messages::GlobalPlannerResultConstPtr& result){
    LOG_INFO<<state.toString().c_str()<<"!";
  }
  void ActiveCallback() {
    LOG_INFO << "Server has recived the goal!";
  }
  void FeedbackCallback(const messages::GlobalPlannerFeedbackConstPtr& feedback){
    if (feedback->error_code != ErrorCode::OK) {
      LOG_INFO<<feedback->error_msg;
    }
    if (!feedback->path.poses.empty()) {
      LOG_INFO<<"Get Path!";
    }
  }
 private:
  ros::Subscriber goal_sub_;
  messages::GlobalPlannerGoal command_;
  bool new_goal_;
  actionlib::SimpleActionClient<messages::GlobalPlannerAction> global_planner_actionlib_client_;
  std::thread thread_;

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_planner_client");
  GlobalPlannerClient global_planner_client;
  ros::spin();
  return 0;
}

