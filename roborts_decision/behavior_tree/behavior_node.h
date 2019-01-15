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

#ifndef ROBORTS_DECISION_BEHAVIOR_NODE_H
#define ROBORTS_DECISION_BEHAVIOR_NODE_H

#include <chrono>
#include <thread>
#include <vector>

#include "../blackboard/blackboard.h"
#include "behavior_state.h"

namespace roborts_decision{
/**
 * @brief Type of behavior tree node
 */
enum class BehaviorType {
  PARALLEL,       ///<Parallel Composite Node
  SELECTOR,       ///<Selector Composite Node
  SEQUENCE,       ///<Sequence Composite Node
  ACTION,         ///<Action Node
  PRECONDITION,   ///<Precondition Node
};
/**
 * @brif Abort Type of behavior tree precondition node
 * @details For more information refer to https://docs.unrealengine.com/en-us/Engine/AI/BehaviorTrees/NodeReference/Decorators
 */
enum class AbortType {
  NONE,           ///<Do not abort anything
  SELF,           ///<Abort self, and any sub-trees running under this node
  LOW_PRIORITY,   ///<Abort any nodes to the right of this node
  BOTH            ///<Abort self, any sub-trees running under me, and any nodes to the right of this node
};
/**
 * @brief Behavior tree base node
 */
class BehaviorNode : public std::enable_shared_from_this<BehaviorNode>{
 public:

  typedef std::shared_ptr<BehaviorNode> Ptr;
  BehaviorNode(){}
  /**
   * @brief Constructor of BehaviorNode
   * @param name Name of the behavior node
   * @param behavior_type Type of the behavior node
   * @param blackboard_ptr Blackboard of the behavior node
   */
  BehaviorNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr):
      name_(name),
      behavior_type_(behavior_type),
      blackboard_ptr_(blackboard_ptr),
      behavior_state_(BehaviorState::IDLE),
      level_(0){}

  virtual ~BehaviorNode()= default;
  /**
   * @brief Run the behavior node
   * @details Roughly including 3 process
   *          1. OnInitilaize: Initialize or reset some variables, when tick the node that is not running.
   *          2. Update: Update and feedback the behavior state.
   *          3. OnTerminate: Reset or recover after getting the result.
   * @return Behavior state
   */
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
  /**
   * @brief Reset the behavior node
   * @details manually invoke the IDLE terminate function when the node is in running status
   */
  virtual void Reset(){
    if (behavior_state_ == BehaviorState::RUNNING){
      behavior_state_ = BehaviorState::IDLE;
      OnTerminate(behavior_state_);
    }
  }
  /**
   * @brief Get the type of the behavior node
   * @return The type of the behavior node
   */
  BehaviorType GetBehaviorType(){
    return behavior_type_;
  }
  /**
   * @brief Get the state of the behavior node
   * @return The state of the behavior node
   */
  BehaviorState GetBehaviorState(){
    return behavior_state_;
  }
  /**
   * @brief Get the name of the behavior node
   * @return The name of the behavior node
   */
  std::string GetName(){
    return name_.c_str();
  }
  /**
   * @brief Set the parent of the behavior node
   * @param parent_node_ptr
   */
  void SetParent(BehaviorNode::Ptr  parent_node_ptr){
    parent_node_ptr_ = parent_node_ptr;
  }
  /**
   * @brief Get the parent node of the behavior node
   * @return The parent node of the behavior node
   */
  BehaviorNode::Ptr GetParent(){
    return parent_node_ptr_ ;
  }
  /**
   * @brief Get the child node of the behavior node
   * @return The child node of the behavior node, for the base class it always returns nullptr
   */
  virtual BehaviorNode::Ptr GetChild(){
    return nullptr;
  }
  /**
   * @brief Get the level of the behavior node
   * @return  The level of the behavior node
   */
  unsigned int GetLevel(){
    return level_;
  }
  /**
   * @brief Set the level of the behavior node
   * @param level The expected level of the behavior node
   */
  void SetLevel(unsigned int level){
    level_ = level;
  }
 protected:
  /**
   * @brief Tick the node and update the state of the behavior node
   * @return the state of the behavior node
   */
  virtual BehaviorState Update() = 0;
  /**
   * @brief Initialize something before starting to tick the node
   */
  virtual void OnInitialize() = 0;
  /**
   * @brief Recover or reset something After getting the result
   * @param state Input behavior state
   */
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
  //! Level of the tree
  unsigned  int level_;
};
/**
 * @brief Behavior tree action node inherited from BehaviorNode
 */
class ActionNode: public BehaviorNode{
 public:
  /**
   * @brief Constructor of ActionNode
   * @param name Name of the behavior node
   * @param blackboard_ptr Blackboard of the behavior node
   */
  ActionNode(std::string name, const Blackboard::Ptr &blackboard_ptr):
      BehaviorNode::BehaviorNode(name,BehaviorType::ACTION, blackboard_ptr){}
  virtual ~ActionNode() = default;

 protected:
  /**
   * @brief Initialize something before starting to tick the node
   */
  virtual void OnInitialize() = 0;
  /**
   * @brief Tick the node and update the state of the behavior node
   * @return the state of the behavior node
   */
  virtual BehaviorState Update() = 0;
  /**
   * @brief Recover or reset something After getting the result
   * @param state Input behavior state
   */
  virtual void OnTerminate(BehaviorState state) = 0;

};
/**
 * @brief Behavior tree decorator node inherited from BehaviorNode
 */
class DecoratorNode: public BehaviorNode{
 public:
  /**
   * @brief Constructor of DecoratorNode
   * @param name Name of the behavior node
   * @param behavior_type Type of the behavior node
   * @param blackboard_ptr Blackboard of the behavior node
   */
  DecoratorNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr):
      BehaviorNode::BehaviorNode(name, behavior_type, blackboard_ptr),
      child_node_ptr_(nullptr){  }
  virtual ~DecoratorNode() = default;
  /**
   * @brief Get the child/decorated node of the behavior node
   * @return the child/decorated node of the behavior node
   */
  virtual BehaviorNode::Ptr GetChild(){
    return child_node_ptr_ ;
  }
  /**
   * @brief Set the child/decorated node of the behavior node
   * @param child_node_ptr the child/decorated node of the behavior node
   */
  void SetChild(const BehaviorNode::Ptr &child_node_ptr) {
    if(child_node_ptr_){
      child_node_ptr_->SetParent(nullptr);
      child_node_ptr->SetLevel(0);
    }
    child_node_ptr_ = child_node_ptr;
    child_node_ptr_->SetParent(shared_from_this());
    child_node_ptr_->SetLevel(level_);
  }

 protected:
  /**
   * @brief Initialize something before starting to tick the node
   */
  virtual void OnInitialize() = 0;
  /**
   * @brief Tick the node and update the state of the behavior node
   * @return the state of the behavior node
   */
  virtual BehaviorState Update() = 0;
  /**
   * @brief Recover or reset something After getting the result
   * @param state Input behavior state
   */
  virtual void OnTerminate(BehaviorState state) = 0;
  //! the child/decorated node of the behavior node
  BehaviorNode::Ptr child_node_ptr_;
};
/**
 * @brief Behavior tree precondition node inherited from DecoratorNode
 */
class PreconditionNode: public DecoratorNode{
 public:
  /**
   * @brief Constructor of PreconditionNode
   * @param name Name of the behavior node
   * @param blackboard_ptr Blackboard of the behavior node
   * @param precondition_function the determined function of the precondition node
   * @param abort_type Abort type of the precondition node
   */
  PreconditionNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
                   std::function<bool()> precondition_function = std::function<bool()>(),
                   AbortType abort_type = AbortType::NONE):
      DecoratorNode::DecoratorNode(name, BehaviorType::PRECONDITION, blackboard_ptr),
      precondition_function_(precondition_function), abort_type_(abort_type){}
  virtual ~PreconditionNode() = default;
  /**
   * @brief Get the abort type of the precondition node
   * @return the abort type of the precondition node
   */
  AbortType GetAbortType(){
    return abort_type_;
  }

 protected:
  /**
   * @brief Initialize something before starting to tick the node
   */
  virtual void OnInitialize() {
    ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
  }

  virtual bool Precondition(){
    if(precondition_function_){
      return precondition_function_();
    }
    else{
      ROS_ERROR("There is no chosen precondition function, then return false by default!");
      return false;
    }
  };
  /**
   * @brief Tick the node and update the state of the behavior node
   * @details Every tick cycle, precondition node will reevaluate the precondition and
   *          tick its decorated node according to the abort type
   * @return the state of the behavior node
   */
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
  /**
   * @brief Recover or reset something After getting the result
   * @param state Input behavior state
   */
  virtual void OnTerminate(BehaviorState state) {
    switch (state){
      case BehaviorState::IDLE:
        ROS_INFO("%s %s IDLE!", name_.c_str(), __FUNCTION__);
        //TODO: the following recovery measure is called by parent node, and deliver to reset its running child node
        child_node_ptr_->Reset();
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("%s %s SUCCESS!", name_.c_str(), __FUNCTION__);
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("%s %s FAILURE!", name_.c_str(), __FUNCTION__);
        //TODO: the following recovery measure is in failure situation caused by precondition false.
        child_node_ptr_->Reset();
        break;
      default:
        ROS_INFO("%s %s ERROR!", name_.c_str(), __FUNCTION__);
        return;
    }
  }
  /**
   * @brief Reevaluate the precondition every tick according to abort type
   * @return True if reevaluation passes
   */
  virtual bool Reevaluation();
  //! the determined function of the precondition node
  std::function<bool()> precondition_function_;
  //! the abort type of the precondition node
  AbortType abort_type_;
};
/**
 * @brief Behavior tree composite node inherited from BehaviorNode
 */
class CompositeNode: public BehaviorNode{
 public:
  /**
   * @brief Constructor of CompositeNode
   * @param name Name of the behavior node
   * @param behavior_type Type of the behavior node
   * @param blackboard_ptr Blackboard of the behavior node
   */
  CompositeNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr):
      BehaviorNode::BehaviorNode(name, behavior_type, blackboard_ptr),
      children_node_index_(0) {
  }

  virtual ~CompositeNode()= default;
  /**
   * @brief Add child behavior node to the composite node
   * @param child_node_ptr The expected child behavior node
   */
  virtual void AddChildren(const BehaviorNode::Ptr& child_node_ptr){
    children_node_ptr_.push_back(child_node_ptr);
    child_node_ptr->SetParent(shared_from_this());
    child_node_ptr->SetLevel(level_+1);
  }
  /**
   * @brief Add a list of child behavior nodes to the composite node
   * @param child_node_ptr A list of the expected child behavior nodes
   */
  virtual void AddChildren(std::initializer_list<BehaviorNode::Ptr> child_node_ptr_list){
    for (auto child_node_ptr = child_node_ptr_list.begin(); child_node_ptr!=child_node_ptr_list.end(); child_node_ptr++) {
      children_node_ptr_.push_back(*child_node_ptr);
      (*child_node_ptr)->SetParent(shared_from_this());
      (*child_node_ptr)->SetLevel(level_+1);
    }
  }
  /**
   * @brief Get the child behavior node that it is turn to tick
   * @return The child behavior node
   */
  virtual BehaviorNode::Ptr GetChild(){
    return children_node_ptr_.at(children_node_index_) ;
  }
  /**
   * @brief Get the list of child behavior nodes
   * @return The list of child behavior nodes
   */
  std::vector<BehaviorNode::Ptr>& GetChildren(){
    return children_node_ptr_;
  }
  /**
   * @brief Get the index of the child behavior node that it is turn to tick
   * @return The index of the child behavior node that it is turn to tick
   */
  unsigned int GetChildrenIndex(){
    return children_node_index_;
  }
  /**
   * @brief Get the number of child behavior nodes
   * @return The number of child behavior nodes
   */
  unsigned int GetChildrenNum(){
    return children_node_ptr_.size();
  }

 protected:
  /**
   * @brief Tick the node and update the state of the behavior node
   * @return the state of the behavior node
   */
  virtual BehaviorState Update() = 0;
  /**
   * @brief Initialize something before starting to tick the node
   */
  virtual void OnInitialize() = 0;
  /**
   * @brief Recover or reset something After getting the result
   * @param state Input behavior state
   */
  virtual void OnTerminate(BehaviorState state) = 0;
  //! the list of child nodes
  std::vector<BehaviorNode::Ptr> children_node_ptr_;
  //! the index of child nodes
  unsigned int children_node_index_;

};
/**
 * @brief Behavior tree selector node inherited from CompositeNode
 */
class SelectorNode: public CompositeNode{
 public:
  /**
   * @brief Constructor of SelectorNode
   * @param name Name of the behavior node
   * @param blackboard_ptr Blackboard of the behavior node
   */
  SelectorNode(std::string name, const Blackboard::Ptr &blackboard_ptr):
      CompositeNode::CompositeNode(name, BehaviorType::SELECTOR, blackboard_ptr) {
  }
  virtual ~SelectorNode() = default;
  /**
   * @brief Overwrite the function in the CompositeNode to add the reevaluation features for selector node
   * @param child_node_ptr The expected child behavior node
   */
  virtual void AddChildren(const BehaviorNode::Ptr& child_node_ptr){

    CompositeNode::AddChildren(child_node_ptr);

    children_node_reevaluation_.push_back
        (child_node_ptr->GetBehaviorType()==BehaviorType::PRECONDITION
             && (std::dynamic_pointer_cast<PreconditionNode>(child_node_ptr)->GetAbortType() == AbortType::LOW_PRIORITY
                 ||std::dynamic_pointer_cast<PreconditionNode>(child_node_ptr)->GetAbortType() == AbortType::BOTH));

  }
  /**
   * @brief Overwrite the function in the CompositeNode to add the reevaluation features for selector node
   * @param child_node_ptr A list of the expected child behavior nodes
   */
  virtual void AddChildren(std::initializer_list<BehaviorNode::Ptr> child_node_ptr_list){

    CompositeNode::AddChildren(child_node_ptr_list);

    for (auto child_node_ptr = child_node_ptr_list.begin(); child_node_ptr!=child_node_ptr_list.end(); child_node_ptr++) {
      children_node_reevaluation_.push_back
          ((*child_node_ptr)->GetBehaviorType()==BehaviorType::PRECONDITION
               && (std::dynamic_pointer_cast<PreconditionNode>(*child_node_ptr)->GetAbortType() == AbortType::LOW_PRIORITY
                   ||std::dynamic_pointer_cast<PreconditionNode>(*child_node_ptr)->GetAbortType() == AbortType::BOTH));
    }
  }
  /**
   * @brief Set the index of the child node to tick
   * @param children_node_index The expected index of the child node to tick
   */
  void SetChildrenIndex(unsigned int children_node_index){
    children_node_index_=children_node_index;
  }
 protected:
  /**
   * @brief Initialize something before starting to tick the node
   */
  virtual void OnInitialize(){
    children_node_index_ = 0;
    ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
  }

  /**
   * @brief Tick the node and update the state of the behavior node
   * @return the state of the behavior node
   */
  virtual BehaviorState Update(){

    if (children_node_ptr_.size() == 0) {
      return BehaviorState::SUCCESS;
    }

    //Reevaluation
    for(unsigned int index = 0; index < children_node_index_; index++){
      ROS_INFO("Reevaluation");
      if (children_node_reevaluation_.at(index)){
        BehaviorState state = children_node_ptr_.at(index)->Run();
        if(index == children_node_index_){
          ROS_INFO("%s abort goes on! ", name_.c_str());
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
  /**
   * @brief Recover or reset something After getting the result
   * @param state Input behavior state
   */
  virtual void OnTerminate(BehaviorState state){
    switch (state){
      case BehaviorState::IDLE:
        ROS_INFO("%s %s IDLE!", name_.c_str(), __FUNCTION__);
        //TODO: the following recovery measure is called by parent node, and deliver to reset its running child node
        children_node_ptr_.at(children_node_index_)->Reset();
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("%s %s SUCCESS!", name_.c_str(), __FUNCTION__);
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("%s %s FAILURE!", name_.c_str(), __FUNCTION__);
        break;
      default:
        ROS_INFO("%s %s ERROR!", name_.c_str(), __FUNCTION__);
        return;
    }
  }

  //! the list of reeavalution state for each child node
  std::vector<bool> children_node_reevaluation_;
};
/**
 * @brief Behavior tree sequence node inherited from CompositeNode
 */
class SequenceNode: public CompositeNode{
 public:
  /**
   * @brief Constructor of SequenceNode
   * @param name Name of behavior node
   * @param blackboard_ptr Blackboard of behavior node
   */
  SequenceNode(std::string name, const Blackboard::Ptr &blackboard_ptr):
      CompositeNode::CompositeNode(name, BehaviorType::SEQUENCE, blackboard_ptr) {}
  virtual ~SequenceNode() = default;

 protected:
  /**
   * @brief Initialize something before starting to tick the node
   */
  virtual void OnInitialize(){
    children_node_index_ = 0;
    ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
  }
  /**
   * @brief Tick the node and update the state of the behavior node
   * @return the state of the behavior node
   */
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
  /**
   * @brief Recover or reset something After getting the result
   * @param state Input behavior state
   */
  virtual void OnTerminate(BehaviorState state){
    switch (state){
      case BehaviorState::IDLE:
        ROS_INFO("%s %s IDLE!", name_.c_str(), __FUNCTION__);
        children_node_ptr_.at(children_node_index_)->Reset();
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("%s %s SUCCESS!", name_.c_str(), __FUNCTION__);
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("%s %s FAILURE!", name_.c_str(), __FUNCTION__);
        break;
      default:
        ROS_INFO("%s %s ERROR!", name_.c_str(), __FUNCTION__);
        return;
    }
  }
};
/**
 * @brief Behavior tree parallel node inherited from CompositeNode
 */
class ParallelNode: public CompositeNode{
 public:
  /**
   * @brief Constructor of ParallelNode
   * @param name Name of the behavior node
   * @param blackboard_ptr Blackboard of the behavior node
   * @param threshold Threshold number of child nodes to determine behavior state of success
   */
  ParallelNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
               unsigned int threshold):
      CompositeNode::CompositeNode(name, BehaviorType::PARALLEL, blackboard_ptr),
      threshold_(threshold),
      success_count_(0),
      failure_count_(0){}

  virtual ~ParallelNode() = default;
 protected:
  /**
   * @brief Initialize something before starting to tick the node
   * @details Initialize and reset the success and failure count of each child node
   */
  virtual void OnInitialize(){
    failure_count_=0;
    success_count_=0;
    children_node_done_.clear();
    children_node_done_.resize(children_node_ptr_.size(),false);
    ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
  }
  /**
   * @brief Tick the node and update the state of the behavior node
   * @return the state of the behavior node
   */
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
  /**
   * @brief Recover or reset something After getting the result
   * @param state Input behavior state
   */
  virtual void OnTerminate(BehaviorState state){
    switch (state){
      case BehaviorState::IDLE:
        ROS_INFO("%s %s IDLE!", name_.c_str(), __FUNCTION__);
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("%s %s SUCCESS!", name_.c_str(), __FUNCTION__);
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("%s %s FAILURE!", name_.c_str(), __FUNCTION__);
        break;
      default:
        ROS_INFO("%s %s ERROR!", name_.c_str(), __FUNCTION__);
        return;
    }
    //TODO: no matter what state, the node would reset all running children to terminate.
    for (unsigned int index = 0; index!=children_node_ptr_.size(); index++) {
      children_node_ptr_.at(index)->Reset();
    }
  }

  //! a list of result checker flags for each child node
  std::vector<bool> children_node_done_;
  //! the current number of child nodes with success behavior state
  unsigned int success_count_;
  //! the current number of child nodes with failure behavior state
  unsigned int failure_count_;
  //! the number of child nodes with success behavior state to determine success of the parallel node
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
      ROS_DEBUG("Can't find current node in parent!");
      return false;
    }
    unsigned int index_in_parent = iter_in_parent - parent_children.begin();
    if (index_in_parent < parent_selector_node_ptr->GetChildrenIndex()){
      if(Precondition()){
        //Abort Measures
        ROS_INFO("Abort happens!");
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

} //namespace roborts_decision


#endif //ROBORTS_DECISION_BEHAVIOR_NODE_H
