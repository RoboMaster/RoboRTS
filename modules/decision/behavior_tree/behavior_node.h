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

enum class BehaviorType {
  PARALLEL,
  SELECTOR,
  SEQUENCE,

  ACTION,

  PRECONDITION,
};
enum class BehaviorState {
  RUNNING,
  SUCCESS,
  FAILURE,
  IDLE,
};
enum class AbortType {
  NONE,
  SELF,
  LOW_PRIORITY,
  BOTH
};

class BehaviorNode : public std::enable_shared_from_this<BehaviorNode>{
 public:

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

    return behavior_state_;
  }

  virtual void Reset(){
    if (behavior_state_ == BehaviorState::RUNNING){
      behavior_state_ = BehaviorState::IDLE;
      OnTerminate(behavior_state_);
    }
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

  void SetParent(BehaviorNode::Ptr  parent_node_ptr){
    parent_node_ptr_ = parent_node_ptr;
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
  //! Parent Node Pointer
  BehaviorNode::Ptr parent_node_ptr_;
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
      child_node_ptr_(child_node_ptr){  }
  virtual ~DecoratorNode() = default;

  void SetChild(const BehaviorNode::Ptr &child_node_ptr) {
    child_node_ptr_ = child_node_ptr;
    child_node_ptr->SetParent(shared_from_this());
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
                   const BehaviorNode::Ptr &child_node_ptr = nullptr,
                   std::function<bool()> precondition_function = std::function<bool()>(),
                   AbortType abort_type = AbortType::NONE):
      DecoratorNode::DecoratorNode(name, BehaviorType::PRECONDITION, blackboard_ptr, child_node_ptr),
      precondition_function_(precondition_function), abort_type_(abort_type){}
  virtual ~PreconditionNode() = default;
  AbortType GetAbortType(){
    return abort_type_;
  }

 protected:

  virtual void OnInitialize() {
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  }
  virtual bool Precondition(){
    if(precondition_function_){
      return precondition_function_();
    }
    else{
      LOG_ERROR<<"There is no chosen precondition function, then return false by default!";
      return false;
    }
  };
  virtual BehaviorState Update(){
    if(child_node_ptr_ == nullptr){
      return BehaviorState::SUCCESS;
    }
    // Reevaluation
    if(Reevaluation()){
      BehaviorState state = child_node_ptr_->Run();
      return state;
    }
    return BehaviorState::FAILURE;
  }
  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" IDLE!";
        //TODO: the following recovery measure is called by parent node, and deliver to reset its running child node
        child_node_ptr_->Reset();
        break;
      case BehaviorState::SUCCESS:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" SUCCESS!";
        break;
      case BehaviorState::FAILURE:
        LOG_INFO<<name_<<" "<<__FUNCTION__<<" FAILURE!";
        //TODO: the following recovery measure is in failure situation caused by precondition false.
        child_node_ptr_->Reset();
        break;
      default:
        LOG_ERROR<<name_<<" "<<__FUNCTION__<<" ERROR!";
        return;
    }
  }
  virtual bool Reevaluation();

  std::function<bool()> precondition_function_;
  AbortType abort_type_;
};

class CompositeNode: public BehaviorNode{
 public:
  CompositeNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr):
      BehaviorNode::BehaviorNode(name, behavior_type, blackboard_ptr),
      children_node_index_(0) {
  }

  virtual ~CompositeNode()= default;

  virtual void AddChildren(const BehaviorNode::Ptr& child_node_ptr){
    children_node_ptr_.push_back(child_node_ptr);
    child_node_ptr->SetParent(shared_from_this());
  }
  virtual void AddChildren(std::initializer_list<BehaviorNode::Ptr> child_node_ptr_list){
    for (auto child_node_ptr = child_node_ptr_list.begin(); child_node_ptr!=child_node_ptr_list.end(); child_node_ptr++) {
      children_node_ptr_.push_back(*child_node_ptr);
      (*child_node_ptr)->SetParent(shared_from_this());
    }

  }
  std::vector<BehaviorNode::Ptr> GetChildren(){
    return children_node_ptr_;
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
  SelectorNode(std::string name, const Blackboard::Ptr &blackboard_ptr):
      CompositeNode::CompositeNode(name, BehaviorType::SELECTOR, blackboard_ptr) {
  }
  virtual ~SelectorNode() = default;

  virtual void AddChildren(const BehaviorNode::Ptr& child_node_ptr){

    CompositeNode::AddChildren(child_node_ptr);

    children_node_reevaluation_.push_back
        (child_node_ptr->GetBehaviorType()==BehaviorType::PRECONDITION
             && (std::dynamic_pointer_cast<PreconditionNode>(child_node_ptr)->GetAbortType() == AbortType::LOW_PRIORITY
                 ||std::dynamic_pointer_cast<PreconditionNode>(child_node_ptr)->GetAbortType() == AbortType::BOTH));

  }
  virtual void AddChildren(std::initializer_list<BehaviorNode::Ptr> child_node_ptr_list){

    CompositeNode::AddChildren(child_node_ptr_list);

    for (auto child_node_ptr = child_node_ptr_list.begin(); child_node_ptr!=child_node_ptr_list.end(); child_node_ptr++) {
      children_node_reevaluation_.push_back
          ((*child_node_ptr)->GetBehaviorType()==BehaviorType::PRECONDITION
               && (std::dynamic_pointer_cast<PreconditionNode>(*child_node_ptr)->GetAbortType() == AbortType::LOW_PRIORITY
                   ||std::dynamic_pointer_cast<PreconditionNode>(*child_node_ptr)->GetAbortType() == AbortType::BOTH));
    }
  }

  void SetChildrenIndex(unsigned int children_node_index){
    children_node_index_=children_node_index;
  }
 protected:
  virtual void OnInitialize(){
    children_node_index_ = 0;
    LOG_INFO<<name_<<" "<<__FUNCTION__;
  };
  virtual BehaviorState Update(){

    if (children_node_ptr_.size() == 0) {
      return BehaviorState::SUCCESS;
    }

    //Reevaluation
    for(unsigned int index = 0; index < children_node_index_; index++){
      LOG_INFO << "Reevaluation";
      if (children_node_reevaluation_.at(index)){
        BehaviorState state = children_node_ptr_.at(index)->Run();
        if(index == children_node_index_){
          LOG_INFO<<name_<<" abort goes on! ";
          if (state != BehaviorState::FAILURE) {
            return state;
          }
          ++children_node_index_;
          break;
        }
      }
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
        //TODO: the following recovery measure is called by parent node, and deliver to reset its running child node
        children_node_ptr_.at(children_node_index_)->Reset();
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
  std::vector<bool> children_node_reevaluation_;
};

class SequenceNode: public CompositeNode{
 public:
  SequenceNode(std::string name, const Blackboard::Ptr &blackboard_ptr):
      CompositeNode::CompositeNode(name, BehaviorType::SEQUENCE, blackboard_ptr) {}
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
        children_node_ptr_.at(children_node_index_)->Reset();
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
               unsigned int threshold):
      CompositeNode::CompositeNode(name, BehaviorType::PARALLEL, blackboard_ptr),
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
    //TODO: no matter what state, the node would reset all running children to terminate.
    for (unsigned int index = 0; index!=children_node_ptr_.size(); index++) {
      children_node_ptr_.at(index)->Reset();
    }
  };
  std::vector<bool> children_node_done_;
  unsigned int success_count_;
  unsigned int failure_count_;
  unsigned int threshold_;
};

bool PreconditionNode::Reevaluation(){

  // Back Reevaluation
  if (parent_node_ptr_ != nullptr && parent_node_ptr_->GetBehaviorType() == BehaviorType::SELECTOR
      && (abort_type_ == AbortType::LOW_PRIORITY || abort_type_ ==  AbortType::BOTH)){
    auto parent_selector_node_ptr = std::dynamic_pointer_cast<SelectorNode>(parent_node_ptr_);

    auto parent_children = parent_selector_node_ptr->GetChildren();
    auto iter_in_parent = std::find(parent_children.begin(), parent_children.end(), shared_from_this());
     if (iter_in_parent == parent_children.end()) {
      LOG_ERROR<< "Can't find current node in parent!";
      return false;
    }
    unsigned int index_in_parent = iter_in_parent - parent_children.begin();
    if (index_in_parent < parent_selector_node_ptr->GetChildrenIndex()){
      if(Precondition()){
        //Abort Measures
        LOG_INFO<<"Abort happens!"<<std::endl;
        parent_children.at(parent_selector_node_ptr->GetChildrenIndex())->Reset();
        parent_selector_node_ptr->SetChildrenIndex(index_in_parent);
        return true;
      }
      else{
        return false;
      }
    }
  }
  // Self Reevaluation

  if(abort_type_== AbortType::SELF || abort_type_== AbortType::BOTH
      || child_node_ptr_->GetBehaviorState() != BehaviorState::RUNNING){
    if(!Precondition()){
      return false;
    }
  }
  return true;
}

} //namespace decision
} //namespace rrts

#endif //MODULE_DECISION_BEHAVIOR_TREE_BEHAVIOR_NODE_H
