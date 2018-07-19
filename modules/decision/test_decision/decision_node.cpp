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

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <thread>
#include "messages/GlobalPlannerAction.h"
#include "messages/LocalPlannerAction.h"

#include "common/error_code.h"
#include "common/io.h"
#include "common/log.h"
#include "common/node_state.h"

#include "modules/perception/map/costmap/costmap_interface.h"

namespace rrts{
namespace decision{

using rrts::common::ErrorCode;
using rrts::common::ErrorInfo;

class DecisionNode {
 public:
  DecisionNode() : global_planner_actionlib_client_("global_planner_node_action", true),
                   local_planner_actionlib_client_("local_planner_node_action", true),
                   rviz_goal_(false), new_path_(false),
                   decision_state_(rrts::common::IDLE) {
    //point mode
    ros::NodeHandle rviz_nh("move_base_simple");
    goal_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1,
                                                              &DecisionNode::GoalCallback, this);


    global_planner_actionlib_client_.waitForServer();
    std::cout << "Global planner module has been connected!" << std::endl;

    local_planner_actionlib_client_.waitForServer();
    std::cout << "Local planner module has been connected!" << std::endl;

    //tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    thread_ = std::thread(&DecisionNode::Execution, this);
  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr & goal){
    global_planner_goal_.goal = *goal;
    rviz_goal_ = true;
  }

  void Execution(){

    while(ros::ok()) {
      if (rviz_goal_) {

        global_planner_actionlib_client_.sendGoal(global_planner_goal_,
                                                  boost::bind(&DecisionNode::GlobalPlannerDoneCallback, this, _1, _2),
                                                  boost::bind(&DecisionNode::GlobalPlannerActiveCallback, this),
                                                  boost::bind(&DecisionNode::GlobalPlannerFeedbackCallback, this, _1)
        );
        decision_state_ = rrts::common::RUNNING;
        rviz_goal_ = false;

      }
      //local goal
      if (new_path_) {

        local_planner_actionlib_client_.sendGoal(local_planner_goal_,
                                                 boost::bind(&DecisionNode::LocalPlannerDoneCallback, this, _1, _2),
                                                 actionlib::SimpleActionClient<messages::LocalPlannerAction>::SimpleActiveCallback(),
                                                 actionlib::SimpleActionClient<messages::LocalPlannerAction>::SimpleFeedbackCallback());
        new_path_ = false;

      }

      usleep(1);
    }

  }

  // Global Planner
  void GlobalPlannerDoneCallback(const actionlib::SimpleClientGoalState& state,  const messages::GlobalPlannerResultConstPtr& result){
    LOG_INFO<<"Global planner "<<state.toString().c_str()<<"!";
    decision_state_ = rrts::common::IDLE;
  }
  void GlobalPlannerActiveCallback(){
    LOG_INFO<<"Global planner server has recived the goal!";
  }
  void GlobalPlannerFeedbackCallback(const messages::GlobalPlannerFeedbackConstPtr& feedback){
    if (feedback->error_code != ErrorCode::OK) {
      LOG_INFO<<"Global planner: "<<feedback->error_msg;
    }
    if (!feedback->path.poses.empty()) {
      local_planner_goal_.route = feedback->path;
      new_path_=true;
    }
  }

  // Local Planner
  void LocalPlannerDoneCallback(const actionlib::SimpleClientGoalState& state,  const messages::LocalPlannerResultConstPtr& result){
    LOG_INFO<<"Local planner "<<state.toString().c_str()<<"!";
    //decision_state_ = rrts::common::IDLE;
  }

  ~DecisionNode(){

    if(thread_.joinable()){
      thread_.join();
    }

  }
 private:
  actionlib::SimpleActionClient<messages::GlobalPlannerAction> global_planner_actionlib_client_;
  actionlib::SimpleActionClient<messages::LocalPlannerAction> local_planner_actionlib_client_;

  messages::GlobalPlannerGoal global_planner_goal_;
  messages::LocalPlannerGoal local_planner_goal_;

  bool enemy_found_;
  bool rviz_goal_;
  bool new_path_;
  std::thread thread_;
  rrts::common::NodeState decision_state_;
  //! costmap
  std::shared_ptr<tf::TransformListener> tf_ptr_;
  std::shared_ptr<rrts::perception::map::CostmapInterface> costmap_ptr_;
  unsigned int gridmap_height_;
  unsigned int gridmap_width_;
  unsigned int size_;
  unsigned char * charmap_;

  ros::Subscriber goal_sub_;
  ros::Publisher enemy_pub_;
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  std::vector<geometry_msgs::PoseStamped>::iterator patrol_goals_iter_;

  //random generate goal
  ros::Subscriber random_sub_;
  unsigned int random_count_;
  float x1_,x2_,y1_,y2_;

};

}// namespace decision
}// namespace rrts

int main(int argc, char **argv) {
  ros::init(argc, argv, "decision_node");
  rrts::decision::DecisionNode global_planner_client;
  ros::spin();
  return 0;
}