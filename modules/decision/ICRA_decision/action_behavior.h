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

#ifndef MODULE_DECISION_ICRA_ACTION_BEHAVIOR_H
#define MODULE_DECISION_ICRA_ACTION_BEHAVIOR_H

#include <unistd.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include "messages/GlobalPlannerAction.h"
#include "messages/LocalPlannerAction.h"

#include "common/error_code.h"
#include "common/log.h"

#include "modules/decision/behavior_tree/behavior_tree.h"
#include "modules/decision/behavior_tree/blackboard.h"

#include "modules/decision/ICRA_decision/goal_factory.h"

namespace rrts {
namespace decision {
class EscapeAction : public ActionNode {
 public:
  EscapeAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("escape_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~EscapeAction() = default;

 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    goal_factory_ptr_->EscapeGoal();

    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelWhirl();
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

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class EscapeAction

class WhirlAction : public ActionNode {
 public:
  WhirlAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("whirl_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~WhirlAction() = default;

 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    return goal_factory_ptr_->Whirl();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelWhirl();
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

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class WhirlAction

class ChaseAction : public ActionNode {
 public:
  ChaseAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("chase_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~ChaseAction() = default;

 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {


    goal_factory_ptr_->ChaseGoal();
    LOG_INFO << "send chase goal";

    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelWhirl();
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

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class ChaseAction

class PatrolAction : public ActionNode {
 public:
  PatrolAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("patrol_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~PatrolAction() = default;

 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    goal_factory_ptr_->PatrolGoal();

    goal_factory_ptr_->UpdateActionState();

    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
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

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class PatrolAction

class SearchAction : public ActionNode {
 public:
  SearchAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
     failure_count_(0) , ActionNode::ActionNode("search_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~SearchAction() = default;

 private:
  virtual void OnInitialize() {
    failure_count_ = 0;
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    if (!(goal_factory_ptr_->SearchValid())) {
      return BehaviorState::FAILURE;
    }

    goal_factory_ptr_->SearchGoal();

    goal_factory_ptr_->UpdateActionState();

    if (goal_factory_ptr_->GetActionState()==BehaviorState::FAILURE) {
      failure_count_++;
      if (failure_count_ == 3) {
        return BehaviorState::FAILURE;
      }
      return BehaviorState::RUNNING;
    } else {
      failure_count_ = 0;
    }

   return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelSearch();
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

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;
  unsigned int failure_count_;

}; // class SearchAction

class GainBuffAction : public ActionNode {
 public:
  GainBuffAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("gain_buff_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~GainBuffAction() = default;

 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    goal_factory_ptr_->BuffGoal();

    goal_factory_ptr_->UpdateActionState();

    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
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

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class GainBuffAction

class ShootAction : public ActionNode {
 public:
  ShootAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("shoot_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~ShootAction() = default;

 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
    blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);

    goal_factory_ptr_->SendGoalTask(blackboard_ptr_->GetEnemy());

    return BehaviorState::RUNNING;
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelWhirl();
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

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class ShootAction

class TurnToWoundedArmorAction : public ActionNode {
 public:
  TurnToWoundedArmorAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("turn_to_wounded_armor_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~TurnToWoundedArmorAction() = default;

 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    goal_factory_ptr_->TurnTOWoundedArmor();

    goal_factory_ptr_->UpdateActionState();

    return goal_factory_ptr_->GetActionState();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelWhirl();
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

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class TurnTOWoundedArmorAction

class TurnToDetectedDirection : public ActionNode {
 public:
  TurnToDetectedDirection(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("turn_to_detected_direction", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~TurnToDetectedDirection() = default;

 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    goal_factory_ptr_->TurnToDetectedDirection();

    goal_factory_ptr_->UpdateActionState();

    return goal_factory_ptr_->GetActionState();

  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
        goal_factory_ptr_->CancelWhirl();
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

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // class TurnToDetectedDirection

class WaitAction : public ActionNode {
 public:

  WaitAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("wait_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~WaitAction() = default;
 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    goal_factory_ptr_->BackBootArea();

    goal_factory_ptr_->UpdateActionState();

    if (goal_factory_ptr_->GetActionState() == BehaviorState::SUCCESS) {
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELAX);
    }
    blackboard_ptr_->ResetAllStatus();

//    if (goal_factory_ptr_->GetState() != BehaviorState::SUCCESS) {
//      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
//      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
//    } else if (goal_factory_ptr_->GetState() == BehaviorState::SUCCESS) {
//    }
    return BehaviorState::RUNNING;
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
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

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // waitAction

class AuxiliaryAction : public ActionNode {
 public:

  AuxiliaryAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
      ActionNode::ActionNode("auxiliary_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {

  }

  virtual ~AuxiliaryAction() = default;
 private:
  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };

  virtual BehaviorState Update() {

    goal_factory_ptr_->GoAuxiliaryPosition();

    goal_factory_ptr_->UpdateActionState();

//    if (goal_factory_ptr_->GetActionState() == BehaviorState::SUCCESS) {
//      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
//      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELAX);
//      blackboard_ptr_->ResetAllStatus();
//    }

//    if (goal_factory_ptr_->GetState() != BehaviorState::SUCCESS) {
//      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
//      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
//    } else if (goal_factory_ptr_->GetState() == BehaviorState::SUCCESS) {
//    }
    if (blackboard_ptr_->GetGameProcess() != GameProcess::FIGHT) {
      return BehaviorState::RUNNING;
    }
    return goal_factory_ptr_->GetActionState();
  }

  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        goal_factory_ptr_->CancelGoal();
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

  GoalFactory::GoalFactoryPtr goal_factory_ptr_;

}; // AuxiliaryAction

}
}

#endif //MODULE_DECISION_ICRA_ACTION_BEHAVIOR_H
