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

#ifndef MODULE_DECISION_BEHAVIOR_TREE_PLANNER_BEHAVIOR_H
#define MODULE_DECISION_BEHAVIOR_TREE_PLANNER_BEHAVIOR_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "messages/GlobalPlannerAction.h"
#include "messages/LocalPlannerAction.h"

#include "common/error_code.h"
#include "common/log.h"

#include "modules/decision/behavior_tree/behavior_tree.h"
#include "modules/decision/behavior_tree/blackboard.h"

namespace rrts{
namespace decision{

class PlannerBehavior: public ActionNode{
 public:
  PlannerBehavior(const Blackboard::Ptr &blackboard_ptr):
      ActionNode::ActionNode("planner_behavior", blackboard_ptr),
      global_planner_actionlib_client_("global_planner_node_action", true),
      local_planner_actionlib_client_("local_planner_node_action", true){
    global_planner_actionlib_client_.waitForServer();
    LOG_INFO<<"Global planer server start!";
    local_planner_actionlib_client_.waitForServer();
    LOG_INFO<<"Local planer server start!";
  }
  virtual ~PlannerBehavior()= default;

 private:
  virtual void OnInitialize() {
    std::cout<<name_<<" "<<__FUNCTION__<<std::endl;
  };
  virtual BehaviorState Update() {
    // First to check if there is a new goal to send
    if (blackboard_ptr_->HasGoal()){
      global_planner_goal_.goal = blackboard_ptr_->GetGoal();
      global_planner_actionlib_client_.sendGoal(global_planner_goal_,
                               actionlib::SimpleActionClient<messages::GlobalPlannerAction>::SimpleDoneCallback(),
                               actionlib::SimpleActionClient<messages::GlobalPlannerAction>::SimpleActiveCallback(),
                               boost::bind(&PlannerBehavior::GlobalPlannerFeedbackCallback, this, _1));
    }

    // Monitor the state to feedback
    while (global_planner_actionlib_client_.getState() == actionlib::SimpleClientGoalState::PENDING) {
      std::this_thread::yield();
    }
    auto state = global_planner_actionlib_client_.getState();
    if(state == actionlib::SimpleClientGoalState::ACTIVE) {
      return BehaviorState::RUNNING;
    }
    else if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return BehaviorState::SUCCESS;
    }
    else if(state == actionlib::SimpleClientGoalState::ABORTED) {
      return BehaviorState::FAILURE;
    }
    else{
        LOG_ERROR<<"error";
    }

  }
  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        global_planner_actionlib_client_.cancelGoal();
        local_planner_actionlib_client_.cancelGoal();
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }

  void GlobalPlannerFeedbackCallback(const messages::GlobalPlannerFeedbackConstPtr& feedback){
    if (!feedback->path.poses.empty()) {
      local_planner_goal_.route = feedback->path;
      local_planner_actionlib_client_.sendGoal(local_planner_goal_);
    }
  }


  actionlib::SimpleActionClient<messages::GlobalPlannerAction> global_planner_actionlib_client_;
  actionlib::SimpleActionClient<messages::LocalPlannerAction> local_planner_actionlib_client_;

  messages::GlobalPlannerGoal global_planner_goal_;
  messages::LocalPlannerGoal local_planner_goal_;

  BehaviorState action_state_;
};

} //namespace decision
} //namespace rrts
#endif //MODULE_DECISION_BEHAVIOR_TREE_PLANNER_BEHAVIOR_H
