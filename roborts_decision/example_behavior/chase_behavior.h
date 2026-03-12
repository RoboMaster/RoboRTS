#ifndef ROBORTS_DECISION_CHASE_BEHAVIOR_H
#define ROBORTS_DECISION_CHASE_BEHAVIOR_H

#include <cmath>
#include <vector>
#include <string>

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {

/**
 * @brief Chase behavior: pursue a detected enemy while maintaining safe distance.
 *
 * Uses a ring buffer of enemy positions to smooth pursuit trajectory.
 * Stops chasing if the enemy is within the engagement range (1.0-2.0m).
 * Falls back along line-of-sight if the goal cell is in an obstacle.
 */
class ChaseBehavior {
 public:
  ChaseBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path)
      : chassis_executor_(chassis_executor),
        blackboard_(blackboard),
        chase_count_(0),
        cancel_goal_(true) {

    chase_goal_.header.frame_id = "map";
    chase_goal_.pose.orientation.x = 0;
    chase_goal_.pose.orientation.y = 0;
    chase_goal_.pose.orientation.z = 0;
    chase_goal_.pose.orientation.w = 1;

    chase_goal_.pose.position.x = 0;
    chase_goal_.pose.position.y = 0;
    chase_goal_.pose.position.z = 0;

    chase_buffer_.resize(kBufferSize);
  }

  void Run() {
    auto executor_state = Update();
    auto robot_map_pose = blackboard_->GetRobotMapPose();

    if (executor_state != BehaviorState::RUNNING) {
      chase_buffer_[chase_count_++ % kBufferSize] = blackboard_->GetEnemy();
      chase_count_ = chase_count_ % kBufferSize;

      const auto& latest_enemy = chase_buffer_[(chase_count_ + kBufferSize - 1) % kBufferSize];
      const double dx = latest_enemy.pose.position.x - robot_map_pose.pose.position.x;
      const double dy = latest_enemy.pose.position.y - robot_map_pose.pose.position.y;
      const double distance = std::sqrt(dx * dx + dy * dy);
      const double yaw = std::atan2(dy, dx);

      // Within engagement range — hold position
      if (distance >= kMinEngageDistance && distance <= kMaxEngageDistance) {
        if (cancel_goal_) {
          chassis_executor_->Cancel();
          cancel_goal_ = false;
        }
        return;
      }

      // Compute approach goal (offset from enemy toward robot)
      geometry_msgs::PoseStamped reduce_goal;
      reduce_goal.pose.orientation = robot_map_pose.pose.orientation;
      reduce_goal.header.frame_id = "map";
      reduce_goal.header.stamp = ros::Time::now();
      reduce_goal.pose.position.x = latest_enemy.pose.position.x - kApproachOffset * std::cos(yaw);
      reduce_goal.pose.position.y = latest_enemy.pose.position.y - kApproachOffset * std::sin(yaw);
      reduce_goal.pose.position.z = 1;

      const double enemy_x = reduce_goal.pose.position.x;
      const double enemy_y = reduce_goal.pose.position.y;

      unsigned int goal_cell_x, goal_cell_y;
      if (!blackboard_->GetCostMap2D()->World2Map(enemy_x, enemy_y,
                                                   goal_cell_x, goal_cell_y)) {
        return;
      }

      // If goal is in obstacle, trace line toward robot to find free cell
      if (blackboard_->GetCostMap2D()->GetCost(goal_cell_x, goal_cell_y) >= kObstacleCostThreshold) {
        unsigned int robot_cell_x, robot_cell_y;
        blackboard_->GetCostMap2D()->World2Map(
            robot_map_pose.pose.position.x,
            robot_map_pose.pose.position.y,
            robot_cell_x, robot_cell_y);

        bool find_goal = false;
        for (FastLineIterator line(goal_cell_x, goal_cell_y, robot_cell_x, robot_cell_y);
             line.IsValid(); line.Advance()) {
          auto point_cost = blackboard_->GetCostMap2D()->GetCost(
              static_cast<unsigned int>(line.GetX()),
              static_cast<unsigned int>(line.GetY()));

          if (point_cost >= kObstacleCostThreshold) {
            continue;
          }

          find_goal = true;
          double goal_x, goal_y;
          blackboard_->GetCostMap2D()->Map2World(
              static_cast<unsigned int>(line.GetX()),
              static_cast<unsigned int>(line.GetY()),
              goal_x, goal_y);

          reduce_goal.pose.position.x = goal_x;
          reduce_goal.pose.position.y = goal_y;
          break;
        }

        if (find_goal) {
          cancel_goal_ = true;
          chassis_executor_->Execute(reduce_goal);
        } else {
          if (cancel_goal_) {
            chassis_executor_->Cancel();
            cancel_goal_ = false;
          }
        }
      } else {
        cancel_goal_ = true;
        chassis_executor_->Execute(reduce_goal);
      }
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  void SetGoal(geometry_msgs::PoseStamped chase_goal) {
    chase_goal_ = chase_goal;
  }

  ~ChaseBehavior() = default;

 private:
  // Configuration constants
  static constexpr size_t kBufferSize = 2;
  static constexpr double kMinEngageDistance = 1.0;   // meters
  static constexpr double kMaxEngageDistance = 2.0;   // meters
  static constexpr double kApproachOffset = 1.2;      // meters offset from enemy
  static constexpr unsigned char kObstacleCostThreshold = 253;

  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! chase goal
  geometry_msgs::PoseStamped chase_goal_;

  //! chase buffer (ring buffer for smoothing)
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;

  //! cancel flag
  bool cancel_goal_;
};

} // namespace roborts_decision

#endif // ROBORTS_DECISION_CHASE_BEHAVIOR_H
