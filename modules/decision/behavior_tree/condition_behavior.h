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

#ifndef MODULE_DECISION_BEHAVIOR_TREE_CONDITION_BEHAVIOR_H
#define MODULE_DECISION_BEHAVIOR_TREE_CONDITION_BEHAVIOR_H
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "messages/GlobalPlannerAction.h"

#include "common/error_code.h"
#include "common/log.h"

#include "modules/decision/behavior_tree/behavior_tree.h"
#include "modules/decision/behavior_tree/blackboard.h"

namespace rrts{
namespace decision {
class EnemyBehavior: public PreconditionNode{
 public:
  EnemyBehavior(const Blackboard::Ptr &blackboard_ptr, const BehaviorNode::Ptr &child_node_ptr = nullptr):
      PreconditionNode("enemy_behavior",blackboard_ptr, child_node_ptr),track_location_(false){

  }
  virtual ~EnemyBehavior() = default;
  virtual void OnInitialize(){
    track_location_ = false;
  }
  virtual bool Precondition(){
    if(blackboard_ptr_->HasEnemy()){
      enemy_ = blackboard_ptr_->GetEnemy();
      blackboard_ptr_->SetGoal(enemy_);
      track_location_=true;
      return true;
    }
    else if(track_location_){
      return true;
    }
    return false;
  }
 private:
  geometry_msgs::PoseStamped enemy_;
  bool track_location_;
};

class PatrolBehavior: public PreconditionNode{
 public:
  PatrolBehavior(const Blackboard::Ptr &blackboard_ptr, const BehaviorNode::Ptr &child_node_ptr = nullptr):
      PreconditionNode("patrol_behavior",blackboard_ptr, child_node_ptr){

  }
  virtual ~PatrolBehavior() = default;
  virtual void OnInitialize(){
    if(!blackboard_ptr_->HasEnemy()){
      patrol_goal_ = blackboard_ptr_->GetPatrol();
      blackboard_ptr_->SetGoal(patrol_goal_);
    }
  }
  virtual bool Precondition(){
    if(!blackboard_ptr_->HasEnemy()){
      return true;
    }
    return false;
  }

 private:
  geometry_msgs::PoseStamped patrol_goal_;
};
} //namespace decision
} //namespace rrts
#endif //MODULE_DECISION_BEHAVIOR_TREE_CONDITION_BEHAVIOR_H
