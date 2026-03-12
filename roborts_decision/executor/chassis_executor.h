#ifndef ROBORTS_DECISION_CHASSIS_EXECUTOR_H
#define ROBORTS_DECISION_CHASSIS_EXECUTOR_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include "roborts_msgs/GlobalPlannerAction.h"
#include "roborts_msgs/LocalPlannerAction.h"
#include "roborts_msgs/TwistAccel.h"
#include "geometry_msgs/Twist.h"

#include "../behavior_tree/behavior_state.h"

namespace roborts_decision {

/**
 * @brief Chassis Executor to execute different abstracted tasks for the chassis module.
 *
 * Supports three execution modes:
 * - GOAL_MODE: Navigation to a target pose via global+local planner
 * - SPEED_MODE: Direct velocity control
 * - SPEED_WITH_ACCEL_MODE: Velocity with acceleration control
 */
class ChassisExecutor {
  using GlobalActionClient = actionlib::SimpleActionClient<roborts_msgs::GlobalPlannerAction>;
  using LocalActionClient = actionlib::SimpleActionClient<roborts_msgs::LocalPlannerAction>;

 public:
  /**
   * @brief Chassis execution mode for different tasks
   */
  enum class ExcutionMode {
    IDLE_MODE,            ///< Default idle mode with no task
    GOAL_MODE,            ///< Goal-targeted task mode using global and local planner
    SPEED_MODE,           ///< Velocity task mode
    SPEED_WITH_ACCEL_MODE ///< Velocity with acceleration task mode
  };

  /**
   * @brief Constructor of ChassisExecutor
   */
  ChassisExecutor();
  ~ChassisExecutor() = default;

  // Non-copyable
  ChassisExecutor(const ChassisExecutor&) = delete;
  ChassisExecutor& operator=(const ChassisExecutor&) = delete;

  /**
   * @brief Execute a goal-targeted task using global and local planner
   * @param goal Given target goal
   */
  void Execute(const geometry_msgs::PoseStamped &goal);

  /**
   * @brief Execute a velocity task
   * @param twist Given velocity
   */
  void Execute(const geometry_msgs::Twist &twist);

  /**
   * @brief Execute a velocity with acceleration task
   * @param twist_accel Given velocity with acceleration
   */
  void Execute(const roborts_msgs::TwistAccel &twist_accel);

  /**
   * @brief Update the current chassis executor state
   * @return Current chassis executor state (same with behavior state)
   */
  BehaviorState Update();

  /**
   * @brief Cancel the current task and return to idle
   */
  void Cancel();

 private:
  /**
   * @brief Global planner feedback callback — forwards path to local planner
   */
  void GlobalPlannerFeedbackCallback(
      const roborts_msgs::GlobalPlannerFeedbackConstPtr& global_planner_feedback);

  //! execution mode of the executor
  ExcutionMode execution_mode_;
  //! execution state of the executor
  BehaviorState execution_state_;

  //! global planner actionlib client
  GlobalActionClient global_planner_client_;
  //! local planner actionlib client
  LocalActionClient local_planner_client_;
  //! global planner actionlib goal
  roborts_msgs::GlobalPlannerGoal global_planner_goal_;
  //! local planner actionlib goal
  roborts_msgs::LocalPlannerGoal local_planner_goal_;

  //! velocity control publisher
  ros::Publisher cmd_vel_pub_;
  //! zero twist for stopping
  geometry_msgs::Twist zero_twist_;

  //! velocity with accel publisher
  ros::Publisher cmd_vel_acc_pub_;
  //! zero twist with acceleration for stopping
  roborts_msgs::TwistAccel zero_twist_accel_;
};

} // namespace roborts_decision

#endif // ROBORTS_DECISION_CHASSIS_EXECUTOR_H