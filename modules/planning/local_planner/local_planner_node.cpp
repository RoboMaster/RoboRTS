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

#include "modules/planning/local_planner/local_planner_node.h"

namespace rrts {
namespace planning {
namespace local_planner {

using rrts::common::NodeState;
LocalPlannerNode::LocalPlannerNode(std::string name) : rrts::common::RRTS::RRTS(name),
                                                       as_(local_planner_nh_, name+"_action", boost::bind(&LocalPlannerNode::ExcuteCB, this, _1), false),
                                                       initialized_(false), node_state_(rrts::common::NodeState::IDLE),
                                                       node_error_info_(rrts::common::ErrorCode::OK), max_error_(5),
                                                       local_cost_(nullptr), tf_(nullptr) {
  if (Init().IsOK()) {
    LOG_INFO<<"local planner initialize completed.";
  } else {
    LOG_WARNING<<"local planner initialize failed.";
    SetNodeState(NodeState::FAILURE);
  }
  as_.start();
}

LocalPlannerNode::~LocalPlannerNode() {
  StopPlanning();
}

rrts::common::ErrorInfo LocalPlannerNode::Init() {
  LOG_INFO << "local planner start";
  LocalAlgorithms local_algorithms;
  rrts::common::ReadProtoFromTextFile("/modules/planning/local_planner/config/local_planner.prototxt", &local_algorithms);
  if (&local_algorithms == nullptr) {
    return rrts::common::ErrorInfo(rrts::common::ErrorCode::LP_INITILIZATION_ERROR,
                                   "Cannot load local planner protobuf configuration file.");
  }
  selected_algorithm_ = local_algorithms.selected_algorithm();
  frequency_ = local_algorithms.frequency();
  tf_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

  local_cost_ = std::make_shared<rrts::perception::map::CostmapInterface>("local_costmap",
                                                                          *tf_,
                                                                          "/modules/perception/map/costmap/config/costmap_parameter_config_for_local_plan.prototxt");
  local_planner_ = rrts::common::AlgorithmFactory<LocalPlannerBase>::CreateAlgorithm(selected_algorithm_);
  if (local_planner_== nullptr) {
    LOG_ERROR<<"global planner algorithm instance can't be loaded";
    return rrts::common::ErrorInfo(rrts::common::ErrorCode::LP_INITILIZATION_ERROR,
                     "local planner algorithm instance can't be loaded");
  }

  std::string name;
  visual_frame_ = local_cost_->GetGlobalFrameID();
  visual_ = LocalVisualizationPtr(new LocalVisualization(local_planner_nh_, visual_frame_));
  vel_pub_ = local_planner_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  return rrts::common::ErrorInfo(rrts::common::ErrorCode::OK);
}

void LocalPlannerNode::ExcuteCB(const messages::LocalPlannerGoal::ConstPtr &command) {
  rrts::common::ErrorInfo error_info = GetErrorInfo();
  NodeState node_state = GetNodeState();

  if (node_state == NodeState::FAILURE) {
    messages::LocalPlannerFeedback feedback;
    messages::LocalPlannerResult result;
    feedback.error_code = error_info.error_code();
    feedback.error_msg  = error_info.error_msg();
    result.error_code   = feedback.error_code;
    as_.publishFeedback(feedback);
    as_.setAborted(result, feedback.error_msg);
    LOG_ERROR << "Initialization Failed, Failed to execute action!";
    return;
  }
  if (plan_mtx_.try_lock()) {
    local_planner_->SetPlan(command->route, local_goal_);
    plan_mtx_.unlock();
    plan_condition_.notify_one();
  }

  LOG_INFO << "Send Plan!";
  if (node_state == NodeState::IDLE) {
    StartPlanning();
  }

  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));

    if (as_.isPreemptRequested()) {
      LOG_INFO <<"Action Preempted";
      if (as_.isNewGoalAvailable()) {
        as_.setPreempted();
        break;
      } else {
        as_.setPreempted();
        StopPlanning();
        break;
      }
    }
    node_state = GetNodeState();
    error_info = GetErrorInfo();

    if (node_state == NodeState::RUNNING|| node_state == NodeState::SUCCESS
        || node_state == NodeState::FAILURE) {
      messages::LocalPlannerFeedback feedback;
      messages::LocalPlannerResult result;
      if (!error_info.IsOK()) {
        feedback.error_code = error_info.error_code();
        feedback.error_msg = error_info.error_msg();
        SetErrorInfo(rrts::common::ErrorInfo::OK());

        as_.publishFeedback(feedback);
      }
      if(node_state == NodeState::SUCCESS) {
        result.error_code = error_info.error_code();
        as_.setSucceeded(result,error_info.error_msg());
        StopPlanning();
        break;
      } else if(node_state == NodeState::FAILURE) {
        result.error_code = error_info.error_code();
        as_.setAborted(result,error_info.error_msg());
        StopPlanning();
        break;
      }
    }
  }

}

void LocalPlannerNode::Loop() {

  rrts::common::ErrorInfo error_info = local_planner_->Initialize(local_cost_, tf_, visual_);
  if (error_info.IsOK()) {
    LOG_INFO<<"local planner algorithm initialize completed.";
  } else {
    LOG_WARNING<<"local planner algorithm initialize failed.";
    SetNodeState(NodeState::FAILURE);
    SetErrorInfo(error_info);
  }
  std::chrono::microseconds sleep_time = std::chrono::microseconds(0);
  int error_count = 0;

  while (GetNodeState() == NodeState::RUNNING) {
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.wait_for(plan_lock, sleep_time);
    auto begin = std::chrono::steady_clock::now();
    rrts::common::ErrorInfo error_info = local_planner_->ComputeVelocityCommands(cmd_vel_);
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin);
    int need_time = 1000 /frequency_;
    sleep_time = std::chrono::milliseconds(need_time) - cost_time;

    if (sleep_time <= std::chrono::milliseconds(0)) {
      //LOG_WARNING << "The time planning once is " << cost_time.count() << " beyond the expected time "
        //        << std::chrono::milliseconds(50).count();
      sleep_time = std::chrono::milliseconds(0);
      //SetErrorInfo(ErrorInfo(ErrorCode::GP_TIME_OUT_ERROR, "Planning once time out."));
    }

    if (error_info.IsOK()) {
      error_count = 0;
      vel_pub_.publish(cmd_vel_);
      if (local_planner_->IsGoalReached()) {
        SetNodeState(NodeState::SUCCESS);
      }
    } else if (error_count > max_error_ && max_error_ >0) {
      LOG_ERROR << "Can not finish plan with max retries( " << max_error_ << " )";
      error_info =  rrts::common::ErrorInfo(rrts::common::ErrorCode::LP_MAX_ERROR_FAILURE, "over max error.");
      SetNodeState(NodeState::FAILURE);
    } else {
      error_count++;
      LOG_ERROR << "Can not get cmd_vel for once. "<< error_info.error_msg() << "error count: " << error_count;
    }

    SetErrorInfo(error_info);
  }

  cmd_vel_.linear.x = 0;
  cmd_vel_.linear.y = 0;
  cmd_vel_.angular.z = 0;
  for (int i = 0; i < 10; ++i) {
    vel_pub_.publish(cmd_vel_);
    usleep(5000);
  }
}

void LocalPlannerNode::SetErrorInfo(const rrts::common::ErrorInfo error_info) {
  std::lock_guard<std::mutex> guard(node_error_info_mtx_);
  node_error_info_ = error_info;
}

void LocalPlannerNode::SetNodeState(const rrts::common::NodeState& node_state) {
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  node_state_ = node_state;
}

rrts::common::NodeState LocalPlannerNode::GetNodeState() {
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  return node_state_;
}

rrts::common::ErrorInfo LocalPlannerNode::GetErrorInfo() {
  std::lock_guard<std::mutex> guard(node_error_info_mtx_);
  return node_error_info_;
}

void LocalPlannerNode::StartPlanning() {
  if (local_planner_thread_.joinable()) {
    local_planner_thread_.join();
  }

  SetNodeState(rrts::common::NodeState::RUNNING);
  local_planner_thread_= std::thread(std::bind(&LocalPlannerNode::Loop,this));
}

void LocalPlannerNode::StopPlanning() {
  SetNodeState(rrts::common::IDLE);
  if (local_planner_thread_.joinable()) {
    local_planner_thread_.join();
  }
}

void LocalPlannerNode::AlgorithmCB(const rrts::common::ErrorInfo &algorithm_error_info) {
  SetErrorInfo(algorithm_error_info);
}

} // namespace local_planner
} // namespace planning
} // namespace rrts

MAIN(rrts::planning::local_planner::LocalPlannerNode, "local_planner_node");
