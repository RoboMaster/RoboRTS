#ifndef ROBORTS_DECISION_GIMBAL_EXECUTOR_H
#define ROBORTS_DECISION_GIMBAL_EXECUTOR_H

#include "ros/ros.h"

#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/GimbalRate.h"

#include "../behavior_tree/behavior_state.h"

namespace roborts_decision {

/**
 * @brief Gimbal Executor to execute different abstracted tasks for the gimbal module.
 *
 * Supports two execution modes:
 * - ANGLE_MODE: Direct angle control
 * - RATE_MODE: Angular rate control
 */
class GimbalExecutor {
 public:
  /**
   * @brief Gimbal execution mode for different tasks
   */
  enum class ExcutionMode {
    IDLE_MODE,   ///< Default idle mode with no task
    ANGLE_MODE,  ///< Angle task mode
    RATE_MODE    ///< Rate task mode
  };

  /**
   * @brief Constructor of GimbalExecutor
   */
  GimbalExecutor();
  ~GimbalExecutor() = default;

  // Non-copyable
  GimbalExecutor(const GimbalExecutor&) = delete;
  GimbalExecutor& operator=(const GimbalExecutor&) = delete;

  /**
   * @brief Execute gimbal angle control
   * @param gimbal_angle Given gimbal angle
   */
  void Execute(const roborts_msgs::GimbalAngle &gimbal_angle);

  /**
   * @brief Execute gimbal rate control
   * @param gimbal_rate Given gimbal rate
   */
  void Execute(const roborts_msgs::GimbalRate &gimbal_rate);

  /**
   * @brief Update the current gimbal executor state
   * @return Current gimbal executor state
   */
  BehaviorState Update();

  /**
   * @brief Cancel the current task and return to idle
   */
  void Cancel();

 private:
  //! execution mode of the executor (note: maintaining original naming for API compat)
  ExcutionMode excution_mode_;
  //! execution state of the executor
  BehaviorState execution_state_;

  //! gimbal rate control publisher
  ros::Publisher cmd_gimbal_rate_pub_;
  //! zero gimbal rate for stopping
  roborts_msgs::GimbalRate zero_gimbal_rate_;

  //! gimbal angle control publisher
  ros::Publisher cmd_gimbal_angle_pub_;
};

} // namespace roborts_decision

#endif // ROBORTS_DECISION_GIMBAL_EXECUTOR_H