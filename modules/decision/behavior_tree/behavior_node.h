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

#ifndef MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_NODE_H
#define MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_NODE_H

#include <chrono>
#include <thread>
#include <vector>

#include "modules/decision/behavior_tree/blackboard.h"

namespace rrts{
namespace decision{

class BehaviorNode {
 public:
  enum BehaviorType {
    PARALLEL,
    SELECTOR,
    SEQUENCE,

    ACTION,

    PRECONDITION,
  };
  enum BehaviorState {
    RUNNING,
    SUCCESS,
    FAILURE,
    IDLE
  };
  typedef std::shared_ptr<BehaviorNode> Ptr;

  BehaviorNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr):
      name_(name),
      behavior_type_(behavior_type),
      blackboard_ptr_(blackboard_ptr),
      behavior_state_(BehaviorState::IDLE) {}

  virtual ~BehaviorNode()= default;

  BehaviorState Run(){

    if (behavior_state_ != BehaviorState::RUNNING) {
      OnInitialize();
    }

    behavior_state_ = Update();

    if (behavior_state_ != BehaviorState::RUNNING) {
      OnTerminate(behavior_state_);
    }
//    std::cout<<name_<<" "<<__FUNCTION__<<behavior_state_;
    return behavior_state_;
  }

  virtual void Reset(){
    behavior_state_ = BehaviorState::IDLE;
    OnTerminate(behavior_state_);
  }

  BehaviorType GetBehaviorType(){
    return behavior_type_;
  }
  BehaviorState GetBehaviorState(){
    return behavior_state_;
  }
  std::string GetName(){
    return name_;
  }

 protected:

  virtual BehaviorState Update() = 0;
  virtual void OnInitialize() = 0;
  virtual void OnTerminate(BehaviorState state) = 0;

  //! Node name
  std::string name_;
  //! State
//  std::mutex behavior_state_mutex_;
  BehaviorState behavior_state_;
  //! Type
  BehaviorType behavior_type_;
  //! Blackboard
  Blackboard::Ptr blackboard_ptr_;
};

class ActionNode: public BehaviorNode{
 public:
  ActionNode(std::string name, const Blackboard::Ptr &blackboard_ptr):
      BehaviorNode::BehaviorNode(name,BehaviorType::ACTION, blackboard_ptr){}
  virtual ~ActionNode() = default;

 protected:
  virtual void OnInitialize() = 0;
  virtual BehaviorState Update() = 0;
  virtual void OnTerminate(BehaviorState state) = 0;

};

class DecoratorNode: public BehaviorNode{
 public:
  DecoratorNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr,
                const BehaviorNode::Ptr &child_node_ptr = nullptr):
      BehaviorNode::BehaviorNode(name, behavior_type, blackboard_ptr),
      child_node_ptr_(child_node_ptr){}
  virtual ~DecoratorNode() = default;

  void SetChild(const BehaviorNode::Ptr &child_node_ptr) {
    child_node_ptr_ = child_node_ptr;
  }

 protected:
  virtual void OnInitialize() = 0;
  virtual BehaviorState Update() = 0;
  virtual void OnTerminate(BehaviorState state) = 0;
  BehaviorNode::Ptr child_node_ptr_;
};

class PreconditionNode: public DecoratorNode{
 public:
  PreconditionNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
                            const BehaviorNode::Ptr &child_node_ptr = nullptr):
      DecoratorNode::DecoratorNode(name, BehaviorType::PRECONDITION, blackboard_ptr, child_node_ptr){}
  virtual ~PreconditionNode() = default;

 protected:

  virtual void OnInitialize() { std::cout<<name_<<" "<<__FUNCTION__; };
  virtual bool Precondition() = 0;
  virtual BehaviorState Update(){
    if(child_node_ptr_ == nullptr){
      return BehaviorState::SUCCESS;
    }
    if(!Precondition()){
      return BehaviorState::FAILURE;
    } else {
      BehaviorState state = child_node_ptr_->Run();
      return state;
    }
  }
  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        if(child_node_ptr_->GetBehaviorState() == BehaviorState::RUNNING){
          child_node_ptr_->Reset();
        }
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        if (child_node_ptr_->GetBehaviorState() == BehaviorState::RUNNING){
          child_node_ptr_->Reset();
        }
        break;
      default:
        LOG_ERROR<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }
};

class CompositeNode: public BehaviorNode{
 public:
  CompositeNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr,
                std::initializer_list<BehaviorNode::Ptr> children_node_ptr = std::initializer_list<BehaviorNode::Ptr>()):
      BehaviorNode::BehaviorNode(name, behavior_type, blackboard_ptr),
      children_node_ptr_(children_node_ptr),
      children_node_index_(0) {}

  virtual ~CompositeNode()= default;

  void AddChildren(const BehaviorNode::Ptr& child_node_ptr){
    children_node_ptr_.push_back(child_node_ptr);
  }
  void AddChildren(std::initializer_list<BehaviorNode::Ptr> child_node_ptr_list){
    children_node_ptr_.insert(children_node_ptr_.end(), child_node_ptr_list.begin(), child_node_ptr_list.end());
  }

  unsigned int GetChildrenIndex(){
    return children_node_index_;
  }
  unsigned int GetChildrenNum(){
    return children_node_ptr_.size();
  }

 protected:
  virtual BehaviorState Update() = 0;
  virtual void OnInitialize() = 0;
  virtual void OnTerminate(BehaviorState state) = 0;


  std::vector<BehaviorNode::Ptr> children_node_ptr_;
  unsigned int children_node_index_;


};

class SelectorNode: public CompositeNode{
 public:
  SelectorNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
                        std::initializer_list<BehaviorNode::Ptr> children_node_ptr = std::initializer_list<BehaviorNode::Ptr>()):
      CompositeNode::CompositeNode(name, BehaviorType::SELECTOR, blackboard_ptr, children_node_ptr) {}
  virtual ~SelectorNode() = default;

 protected:
  virtual void OnInitialize(){
    children_node_index_ = 0;
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };
  virtual BehaviorState Update(){

    if (children_node_ptr_.size() == 0) {
      return BehaviorState::SUCCESS;
    }

    while(true){

      BehaviorState state = children_node_ptr_.at(children_node_index_)->Run();

      if (state != BehaviorState::FAILURE) {
        return state;
      }
      if (++children_node_index_ == children_node_ptr_.size()) {
        children_node_index_ = 0;
        return BehaviorState::FAILURE;
      }

    }
  }
  virtual void OnTerminate(BehaviorState state){
    switch (state){
      case BehaviorState::IDLE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        if(children_node_ptr_.at(children_node_index_)->GetBehaviorState() == BehaviorState::RUNNING){
          children_node_ptr_.at(children_node_index_)->Reset();
        }

        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_ERROR<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }
};

class SequenceNode: public CompositeNode{
 public:
  SequenceNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
                        std::initializer_list<BehaviorNode::Ptr> children_node_ptr = std::initializer_list<BehaviorNode::Ptr>()):
      CompositeNode::CompositeNode(name, BehaviorType::SEQUENCE, blackboard_ptr, children_node_ptr) {}
  virtual ~SequenceNode() = default;

 protected:
  virtual void OnInitialize(){
    children_node_index_ = 0;
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };
  virtual BehaviorState Update(){

    if (children_node_ptr_.size() == 0) {
      return BehaviorState::SUCCESS;
    }

    while(true){

      BehaviorState state = children_node_ptr_.at(children_node_index_)->Run();

      if (state != BehaviorState::SUCCESS) {
        return state;
      }
      if (++children_node_index_ == children_node_ptr_.size()) {
        children_node_index_ = 0;
        return BehaviorState::SUCCESS;
      }

    }
  }
  virtual void OnTerminate(BehaviorState state){
    switch (state){
      case BehaviorState::IDLE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        if( children_node_ptr_.at(children_node_index_)->GetBehaviorState() == BehaviorState::RUNNING){
          children_node_ptr_.at(children_node_index_)->Reset();
        }
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_ERROR<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }
};

class ParallelNode: public CompositeNode{
 public:
  ParallelNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
               unsigned int threshold,
               std::initializer_list<BehaviorNode::Ptr> children_node_ptr = std::initializer_list<BehaviorNode::Ptr>()):
      CompositeNode::CompositeNode(name, BehaviorType::PARALLEL, blackboard_ptr, children_node_ptr),
      threshold_(threshold),
      success_count_(0),
      failure_count_(0){}

  virtual ~ParallelNode() = default;
 protected:

  virtual void OnInitialize(){
    failure_count_=0;
    success_count_=0;
    children_node_done_.clear();
    children_node_done_.resize(children_node_ptr_.size(),false);

    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };
  virtual BehaviorState Update(){

    if (children_node_ptr_.size() == 0) {
      return BehaviorState::SUCCESS;
    }

    for (unsigned int index = 0; index!=children_node_ptr_.size(); index++) {
      if (children_node_done_.at(index) == false){
        BehaviorState state = children_node_ptr_.at(index)->Run();

        if (state == BehaviorState::SUCCESS) {
          children_node_done_.at(index) = true;
          if (++success_count_ >= threshold_) {
            return BehaviorState::SUCCESS;
          }
        }
        else if (state == BehaviorState::FAILURE) {
          children_node_done_.at(index) = true;
          if (++failure_count_ >= children_node_ptr_.size()-threshold_) {
            return BehaviorState::FAILURE;
          }
        }

      }
    }
    return BehaviorState::RUNNING;
  }
  virtual void OnTerminate(BehaviorState state){
    switch (state){
      case BehaviorState::IDLE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        break;
      default:
        LOG_ERROR<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
    for (unsigned int index = 0; index!=children_node_ptr_.size(); index++) {
      if (children_node_ptr_.at(index)->GetBehaviorState() == BehaviorState::RUNNING){
        children_node_ptr_.at(index)->Reset();
      }
    }
  };
  std::vector<bool> children_node_done_;
  unsigned int success_count_;
  unsigned int failure_count_;
  unsigned int threshold_;
};

} //namespace decision
} //namespace rrts

#endif //MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_NODE_H
