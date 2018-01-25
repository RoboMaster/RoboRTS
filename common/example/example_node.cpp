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
#include "common/example/example_node.h"
#include "common/main_interface.h"

namespace rrts{
namespace common {

MainTest::MainTest(std::string name):RRTS::RRTS(name),
                                     as_(nh_, "test_action", boost::bind(&MainTest::ActionCB,this,_1), false),
                                     running_(false),
                                     initialized_(false),
                                     node_state_(NodeState::IDLE),
                                     return_state_(ErrorCode::OK) {
  // Param settings (protobuf files, including selected_algorithm)
  cycle_duration_= std::chrono::microseconds((int)(1e6 / 1));
  // ROS settings
  ros::NodeHandle nh;
  sub_=nh.subscribe<geometry_msgs::PoseStamped>("test_topic",10, boost::bind(&MainTest::CB,this,_1));
  // Algorithm instance
  rrts::common::REGISTER_ALGORITHM(ExampleBase, "example_algorithm", ExampleAlgorithm, int);
  selected_algorithm_ptr_ = rrts::common::AlgorithmFactory<ExampleBase,int>::CreateAlgorithm("example_algorithm",5);
  // Start all and set it initialized;
  LOG_INFO<<__FUNCTION__<<" initialized!";

  initialized_ = true;
  as_.start();
}
MainTest::~MainTest(){
  StopThread();
}
void MainTest::ActionCB(const messages::exampleGoal::ConstPtr &command){

  LOG_INFO<<__FUNCTION__<<" start!";

  messages::exampleFeedback feedback;
  messages::exampleResult result;
  if(!initialized_){
    feedback.error_code = GetReturnState().error_code();
    feedback.error_msg = GetReturnState().error_msg();
    result.error_code = feedback.error_code;
    as_.publishFeedback(feedback);
    as_.setAborted(result, feedback.error_msg);
    LOG_INFO<<"Initialization Failed, Failed to execute action!";
    return;
  }

  if(GetNodeState() == NodeState::IDLE){
    StartThread();
  }

  while (ros::ok()) {
    std::cout<<"0"<<std::endl;
    if (as_.isPreemptRequested())
    {
      LOG_INFO<<"Action Preempted";

      as_.setPreempted();
      SetNodeState(NodeState::IDLE);
      StopThread();
    }

    if(GetNodeState() == NodeState::RUNNING){

      feedback.error_code = GetReturnState().error_code();
      feedback.error_msg = GetReturnState().error_msg();
      as_.publishFeedback(feedback);

    }

    else if(GetNodeState() == NodeState::FAILURE) {

      feedback.error_code = GetReturnState().error_code();
      feedback.error_msg = GetReturnState().error_msg();
      result.error_code = feedback.error_code;

      as_.publishFeedback(feedback);
      as_.setAborted(result, feedback.error_msg);

      SetNodeState(NodeState::IDLE);
      LOG_INFO<<"Action Failed!";
      break;
    }
    else if(GetNodeState() == NodeState::SUCCESS){
      feedback.error_code = GetReturnState().error_code();
      feedback.error_msg = GetReturnState().error_msg();
      result.error_code = feedback.error_code;

      as_.publishFeedback(feedback);
      as_.setSucceeded(result);

      SetNodeState(NodeState::IDLE);
      LOG_INFO<<"Action Succeeded!";
      break;
    }
  }

  LOG_INFO<<__FUNCTION__<<" Terminate!";

}

void MainTest::CB(const geometry_msgs::PoseStamped::ConstPtr & pose){
  LOG_INFO<<__FUNCTION__<<" Start!";
}
void MainTest::SetNodeState(const NodeState& node_state) {
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  node_state_ = node_state;
}
NodeState MainTest::GetNodeState() {
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  return node_state_;
}
void MainTest::SetReturnState(const ErrorInfo return_state) {
  std::lock_guard<std::mutex> guard(return_state_mtx_);
  return_state_ = return_state;
}
ErrorInfo MainTest::GetReturnState() {
  std::lock_guard<std::mutex> guard(return_state_mtx_);
  return return_state_;
}

void MainTest::StartThread(){
  LOG_INFO<<__FUNCTION__<<" Start!";

  if(thread_.joinable()){thread_.join();}
  running_ = true;
  thread_= std::thread(std::bind(&MainTest::FunctionThread,this));
}

void MainTest::StopThread(){
  running_ = false;
  if(thread_.joinable()) {
    LOG_INFO<<__FUNCTION__<<" Called!";
    thread_.join();
  }
}

void MainTest::FunctionThread(){
  LOG_INFO<<__FUNCTION__<<" Start!";

  int count = 0;
  while(running_){

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    SetNodeState(NodeState::RUNNING);

    ErrorInfo return_state = selected_algorithm_ptr_->Function();

    SetReturnState(return_state);
    if(!return_state.IsOK()){
      SetNodeState(NodeState::FAILURE);
      running_ = false;
    }
    else if(++count>10){
      SetNodeState(NodeState::SUCCESS);
      running_ = false;
    } else{
      std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
      std::chrono::microseconds execution_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
      std::chrono::microseconds sleep_time = cycle_duration_ - execution_duration;
      LOG_INFO<< static_cast<double>(sleep_time.count())* std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
      std::this_thread::sleep_for(sleep_time);
    }
  }

  LOG_INFO<<__FUNCTION__<<" Terminate!";
}

}
}

MAIN(rrts::common::MainTest,"test_node");