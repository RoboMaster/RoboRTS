#ifndef ROBORTS_DECISION_GOAL_BEHAVIOR_H
#define ROBORTS_DECISION_GOAL_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"

namespace roborts_decision {

/**
 * @brief Goal behavior: navigate to a goal set via RViz or other external source.
 *
 * Monitors the blackboard for new goals and sends them to the chassis executor.
 * This is the simplest behavior — it just forwards external goals.
 */
class GoalBehavior {
 public:
  GoalBehavior(ChassisExecutor* &chassis_executor,
               Blackboard* &blackboard)
      : chassis_executor_(chassis_executor),
        blackboard_(blackboard) {}

  void Run() {
    if (blackboard_->IsNewGoal()) {
      auto goal = blackboard_->GetGoal();
      ROS_INFO("GoalBehavior: navigating to (%.2f, %.2f)",
               goal.pose.position.x, goal.pose.position.y);
      chassis_executor_->Execute(goal);
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~GoalBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
};

} // namespace roborts_decision

#endif // ROBORTS_DECISION_GOAL_BEHAVIOR_H
