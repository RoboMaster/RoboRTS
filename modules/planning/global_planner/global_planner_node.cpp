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

#include "modules/planning/global_planner/global_planner_node.h"
#include "common/main_interface.h"

namespace rrts {
namespace planning {
namespace global_planner {

using rrts::common::ErrorCode;
using rrts::common::ErrorInfo;
using rrts::common::NodeState;
GlobalPlannerNode::GlobalPlannerNode(std::string name) :
    rrts::common::RRTS::RRTS(name),
    new_path_(false),pause_(false), node_state_(NodeState::IDLE), error_info_(ErrorCode::OK),
    as_(nh_,"global_planner_node_action",boost::bind(&GlobalPlannerNode::GoalCallback,this,_1),false) {

  if (Init().IsOK()) {
    LOG_INFO<<"Initialization completed.";
    StartPlanning();
    as_.start();
  } else {
    LOG_WARNING<<"Initialization failed.";
    SetNodeState(NodeState::FAILURE);
  }

}

ErrorInfo GlobalPlannerNode::Init() {

  //Load proto planning configuration parameters
  GlobalPlannerConfig global_planner_config;
  if (!rrts::common::ReadProtoFromTextFile("/modules/planning/global_planner/config/global_planner_config.prototxt",
                                           &global_planner_config)) {
    LOG_ERROR<<"Cannot load global planner protobuf configuration file.";
    return ErrorInfo(ErrorCode::GP_INITILIZATION_ERROR,
                     "Cannot load global planner protobuf configuration file.");
  }

  selected_algorithm_ = global_planner_config.selected_algorithm();
  cycle_duration_ = std::chrono::microseconds((int) (1e6 / global_planner_config.frequency()));
  max_retries_ = global_planner_config.max_retries();
  goal_distance_tolerance_ = global_planner_config.goal_distance_tolerance();
  goal_angle_tolerance_ = global_planner_config.goal_angle_tolerance();

  //ROS Visualize
  path_pub_ = nh_.advertise<nav_msgs::Path>("global_planner_path", 10);

  //Create the instance of the selected algorithm
  tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

  costmap_ptr_ = std::make_shared<rrts::perception::map::CostmapInterface>("global_costmap",
                                                                           *tf_ptr_,
                                                                           "/modules/perception/map/costmap/config/costmap_parameter_config_for_global_plan.prototxt");
  global_planner_ptr_ = rrts::common::AlgorithmFactory<GlobalPlannerBase,CostmapPtr >::CreateAlgorithm(
      selected_algorithm_, costmap_ptr_);
  if (global_planner_ptr_== nullptr) {
    LOG_ERROR<<"global planner algorithm instance can't be loaded";
    return ErrorInfo(ErrorCode::GP_INITILIZATION_ERROR,
                     "global planner algorithm instance can't be loaded");
  }
  path_.header.frame_id = costmap_ptr_->GetGlobalFrameID();
  return ErrorInfo(ErrorCode::OK);
}

void GlobalPlannerNode::GoalCallback(const messages::GlobalPlannerGoal::ConstPtr &msg) {
  LOG_INFO<<"Received a Goal from client!";

  ErrorInfo error_info = GetErrorInfo();
  NodeState node_state = GetNodeState();

//  if (node_state == NodeState::FAILURE) {
//    messages::GlobalPlannerFeedback feedback;
//    messages::GlobalPlannerResult result;
//    feedback.error_code = error_info.error_code();
//    feedback.error_msg = error_info.error_msg();
//    result.error_code = feedback.error_code;
//    as_.publishFeedback(feedback);
//    as_.setAborted(result,feedback.error_msg);
//    LOG_ERROR << "Initialization Failed, Failed to execute action!";
//    return;
//  }

  SetGoal(msg->goal);

  if (GetNodeState() != NodeState::RUNNING) {
    SetNodeState(NodeState::RUNNING);
  }

  {
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.notify_one();
  }



  while (ros::ok()) {

    if (as_.isPreemptRequested()) {
      if (as_.isNewGoalAvailable()) {
        as_.setPreempted();
        LOG_INFO<<"Override!";
        break;
      }else{
        as_.setPreempted();
        SetNodeState(NodeState::IDLE);
        LOG_INFO<<"Cancel!";
        break;
      }
    }

    node_state = GetNodeState();
    error_info = GetErrorInfo();

    if(node_state == NodeState::RUNNING || node_state == NodeState::SUCCESS || node_state == NodeState::FAILURE) {
      messages::GlobalPlannerFeedback feedback;
      messages::GlobalPlannerResult result;
      if (!error_info.IsOK() || new_path_) {
        if (!error_info.IsOK()) {
          feedback.error_code = error_info.error_code();
          feedback.error_msg = error_info.error_msg();
          SetErrorInfo(ErrorInfo::OK());
        }
        if (new_path_) {
          feedback.path = path_;
          new_path_ = false;
        }
        as_.publishFeedback(feedback);
      }
      if(node_state == NodeState::SUCCESS){
        result.error_code = error_info.error_code();
        as_.setSucceeded(result,error_info.error_msg());
        SetNodeState(NodeState::IDLE);
        break;
      }
      else if(node_state == NodeState::FAILURE){
        result.error_code = error_info.error_code();
        as_.setAborted(result,error_info.error_msg());
        SetNodeState(NodeState::IDLE);
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }

}

NodeState GlobalPlannerNode::GetNodeState() {
  std::lock_guard<std::mutex> node_state_lock(node_state_mtx_);
  return node_state_;
}

void GlobalPlannerNode::SetNodeState(NodeState node_state) {
  std::lock_guard<std::mutex> node_state_lock(node_state_mtx_);
  node_state_ = node_state;
}

ErrorInfo GlobalPlannerNode::GetErrorInfo() {
  std::lock_guard<std::mutex> error_info_lock(error_info_mtx_);
  return error_info_;
}

void GlobalPlannerNode::SetErrorInfo(ErrorInfo error_info) {
  std::lock_guard<std::mutex> node_state_lock(error_info_mtx_);
  error_info_ = error_info;
}

geometry_msgs::PoseStamped GlobalPlannerNode::GetGoal() {
  std::lock_guard<std::mutex> goal_lock(goal_mtx_);
  return goal_;
}

void GlobalPlannerNode::SetGoal(geometry_msgs::PoseStamped goal) {
  std::lock_guard<std::mutex> goal_lock(goal_mtx_);
  goal_ = goal;
}

void GlobalPlannerNode::StartPlanning() {
  SetNodeState(NodeState::IDLE);
  plan_thread_ = std::thread(&GlobalPlannerNode::PlanThread, this);
}

void GlobalPlannerNode::StopPlanning() {
  SetNodeState(NodeState::RUNNING);
  if (plan_thread_.joinable()) {
    plan_thread_.join();
  }
}

void GlobalPlannerNode::PlanThread() {
  LOG_INFO<<"Plan thread start!";
  geometry_msgs::PoseStamped current_start;
  geometry_msgs::PoseStamped current_goal;
  std::vector<geometry_msgs::PoseStamped> current_path;
  std::chrono::microseconds sleep_time = std::chrono::microseconds(0);
  ErrorInfo error_info;
  int retries = 0;
  while (ros::ok()) {
    LOG_INFO<<"Wait to plan!";
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.wait_for(plan_lock, sleep_time);
    while (GetNodeState()!=NodeState::RUNNING){
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    LOG_INFO<<"Go on planning!";

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    {
      std::unique_lock<rrts::perception::map::Costmap2D::mutex_t> lock(*(costmap_ptr_->GetCostMap()->GetMutex()));
      bool error_set = false;
      while (!costmap_ptr_->GetRobotPose(current_start)) {
        if (!error_set) {
          LOG_ERROR<<"Get Robot Pose Error.";
          SetErrorInfo(ErrorInfo(ErrorCode::GP_GET_POSE_ERROR, "Get Robot Pose Error."));
          error_set = true;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1));
      }

      current_goal = GetGoal();

      if (current_goal.header.frame_id != costmap_ptr_->GetGlobalFrameID()) {
        current_goal = costmap_ptr_->Pose2GlobalFrame(current_goal);
        SetGoal(current_goal);
      }

      error_info = global_planner_ptr_->Plan(current_start, current_goal, current_path);

    }

    if (error_info.IsOK()) {
      retries = 0;
      PathVisualization(current_path);

      current_goal = current_path.back();
      SetGoal(current_goal);
      if (GetDistance(current_start, current_goal) < goal_distance_tolerance_
          && GetAngle(current_start, current_goal) < goal_angle_tolerance_
          ) {
        SetNodeState(NodeState::SUCCESS);
      }
    } else if (max_retries_ > 0 && retries > max_retries_) {
      LOG_ERROR << "Can not get plan with max retries( " << max_retries_ << " )";
      error_info = ErrorInfo(ErrorCode::GP_MAX_RETRIES_FAILURE, "Over max retries.");
      SetNodeState(NodeState::FAILURE);
      retries=0;
    } else if (error_info == ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR)){
      LOG_ERROR << "Current goal is not valid!";
      SetNodeState(NodeState::FAILURE);
      retries=0;
    }
    else {
      retries++;
      LOG_ERROR << "Can not get plan for once. " << error_info.error_msg();
    }

    SetErrorInfo(error_info);

    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::chrono::microseconds execution_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    sleep_time = cycle_duration_ - execution_duration;

    if (sleep_time <= std::chrono::microseconds(0)) {
      LOG_ERROR << "The time planning once is " << execution_duration.count() << " beyond the expected time "
                << cycle_duration_.count();
      sleep_time = std::chrono::microseconds(0);
      SetErrorInfo(ErrorInfo(ErrorCode::GP_TIME_OUT_ERROR, "Planning once time out."));
    }
  }


  LOG_INFO << "Plan thread terminated!";
}

void GlobalPlannerNode::PathVisualization(const std::vector<geometry_msgs::PoseStamped> &path) {
  path_.poses = path;
  path_pub_.publish(path_);
  new_path_ = true;
}

double GlobalPlannerNode::GetDistance(const geometry_msgs::PoseStamped &pose1,
                                      const geometry_msgs::PoseStamped &pose2) {
  const geometry_msgs::Point point1 = pose1.pose.position;
  const geometry_msgs::Point point2 = pose2.pose.position;
  const double dx = point1.x - point2.x;
  const double dy = point1.y - point2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double GlobalPlannerNode::GetAngle(const geometry_msgs::PoseStamped &pose1,
                                   const geometry_msgs::PoseStamped &pose2) {
  const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
  const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
  tf::Quaternion rot1, rot2;
  tf::quaternionMsgToTF(quaternion1, rot1);
  tf::quaternionMsgToTF(quaternion2, rot2);
  return rot1.angleShortestPath(rot2);
}

GlobalPlannerNode::~GlobalPlannerNode() {
  StopPlanning();
}

} //namespace global_planner
} //namespace planning
} //namespace rrts

MAIN(rrts::planning::global_planner::GlobalPlannerNode, "global_planner_node")
