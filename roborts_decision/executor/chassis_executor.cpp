#include <actionlib/client/simple_action_client.h>
#include <chrono>
#include <thread>
#include "chassis_executor.h"

namespace roborts_decision {

namespace {
  constexpr int kCancelSleepMs = 50;  // sleep after canceling accel mode
  constexpr int kCmdVelAccQueueSize = 100;
  constexpr int kCmdVelQueueSize = 1;
} // anonymous namespace

ChassisExecutor::ChassisExecutor()
    : execution_mode_(ExcutionMode::IDLE_MODE),
      execution_state_(BehaviorState::IDLE),
      global_planner_client_("global_planner_node_action", true),
      local_planner_client_("local_planner_node_action", true) {
  ros::NodeHandle nh;
  cmd_vel_acc_pub_ = nh.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc", kCmdVelAccQueueSize);
  cmd_vel_pub_     = nh.advertise<geometry_msgs::Twist>("cmd_vel", kCmdVelQueueSize);

  ROS_INFO("Waiting for global planner action server...");
  global_planner_client_.waitForServer();
  ROS_INFO("Global planner server connected.");

  ROS_INFO("Waiting for local planner action server...");
  local_planner_client_.waitForServer();
  ROS_INFO("Local planner server connected.");
}

void ChassisExecutor::Execute(const geometry_msgs::PoseStamped &goal) {
  execution_mode_ = ExcutionMode::GOAL_MODE;
  global_planner_goal_.goal = goal;
  global_planner_client_.sendGoal(
      global_planner_goal_,
      GlobalActionClient::SimpleDoneCallback(),
      GlobalActionClient::SimpleActiveCallback(),
      boost::bind(&ChassisExecutor::GlobalPlannerFeedbackCallback, this, _1));
}

void ChassisExecutor::Execute(const geometry_msgs::Twist &twist) {
  if (execution_mode_ == ExcutionMode::GOAL_MODE) {
    Cancel();
  }
  execution_mode_ = ExcutionMode::SPEED_MODE;
  cmd_vel_pub_.publish(twist);
}

void ChassisExecutor::Execute(const roborts_msgs::TwistAccel &twist_accel) {
  if (execution_mode_ == ExcutionMode::GOAL_MODE) {
    Cancel();
  }
  execution_mode_ = ExcutionMode::SPEED_WITH_ACCEL_MODE;
  cmd_vel_acc_pub_.publish(twist_accel);
}

BehaviorState ChassisExecutor::Update() {
  actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::LOST;

  switch (execution_mode_) {
    case ExcutionMode::IDLE_MODE:
      execution_state_ = BehaviorState::IDLE;
      break;

    case ExcutionMode::GOAL_MODE:
      state = global_planner_client_.getState();
      if (state == actionlib::SimpleClientGoalState::ACTIVE) {
        ROS_DEBUG("%s: ACTIVE", __FUNCTION__);
        execution_state_ = BehaviorState::RUNNING;
      } else if (state == actionlib::SimpleClientGoalState::PENDING) {
        ROS_DEBUG("%s: PENDING", __FUNCTION__);
        execution_state_ = BehaviorState::RUNNING;
      } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("%s: SUCCEEDED", __FUNCTION__);
        execution_state_ = BehaviorState::SUCCESS;
      } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_WARN("%s: ABORTED", __FUNCTION__);
        execution_state_ = BehaviorState::FAILURE;
      } else {
        ROS_ERROR("Unexpected planner state: %s", state.toString().c_str());
        execution_state_ = BehaviorState::FAILURE;
      }
      break;

    case ExcutionMode::SPEED_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    case ExcutionMode::SPEED_WITH_ACCEL_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    default:
      ROS_ERROR("Unknown Execution Mode");
  }

  return execution_state_;
}

void ChassisExecutor::Cancel() {
  switch (execution_mode_) {
    case ExcutionMode::IDLE_MODE:
      ROS_DEBUG("Nothing to cancel — already idle.");
      break;

    case ExcutionMode::GOAL_MODE:
      global_planner_client_.cancelGoal();
      local_planner_client_.cancelGoal();
      execution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::SPEED_MODE:
      cmd_vel_pub_.publish(zero_twist_);
      execution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::SPEED_WITH_ACCEL_MODE:
      cmd_vel_acc_pub_.publish(zero_twist_accel_);
      execution_mode_ = ExcutionMode::IDLE_MODE;
      // Brief sleep to ensure the zero-velocity command is processed
      std::this_thread::sleep_for(std::chrono::milliseconds(kCancelSleepMs));
      break;

    default:
      ROS_ERROR("Unknown Execution Mode");
  }
}

void ChassisExecutor::GlobalPlannerFeedbackCallback(
    const roborts_msgs::GlobalPlannerFeedbackConstPtr& global_planner_feedback) {
  if (!global_planner_feedback->path.poses.empty()) {
    local_planner_goal_.route = global_planner_feedback->path;
    local_planner_client_.sendGoal(local_planner_goal_);
  }
}

} // namespace roborts_decision