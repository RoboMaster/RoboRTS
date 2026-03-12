#ifndef ROBORTS_DECISION_ESCAPEBEHAVIOR_H
#define ROBORTS_DECISION_ESCAPEBEHAVIOR_H

#include "io/io.h"
#include "roborts_msgs/TwistAccel.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {

/**
 * @brief Escape behavior: flee from a detected enemy to a position behind obstacles.
 *
 * Selects a random goal on the opposite side of the field from the enemy,
 * validates that there are sufficient obstacles blocking line-of-sight,
 * and navigates to that position. Falls back to spinning in place if
 * no enemy is detected or the enemy position can't be mapped.
 */
class EscapeBehavior {
 public:
  EscapeBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path)
      : chassis_executor_(chassis_executor),
        blackboard_(blackboard) {

    // Initialize whirl velocity to zero
    whirl_vel_.accel.linear.x = 0;
    whirl_vel_.accel.linear.y = 0;
    whirl_vel_.accel.linear.z = 0;
    whirl_vel_.accel.angular.x = 0;
    whirl_vel_.accel.angular.y = 0;
    whirl_vel_.accel.angular.z = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s: failed to load config file", __FUNCTION__);
    }
  }

  void Run() {
    auto executor_state = Update();

    if (executor_state != BehaviorState::RUNNING) {
      if (blackboard_->IsEnemyDetected()) {
        geometry_msgs::PoseStamped enemy = blackboard_->GetEnemy();
        auto robot_map_pose = blackboard_->GetRobotMapPose();

        // Determine escape region based on enemy position
        float x_min, x_max;
        if (enemy.pose.position.x < left_x_limit_) {
          x_min = right_random_min_x_;
          x_max = right_random_max_x_;
        } else if (enemy.pose.position.x > right_x_limit_) {
          x_min = left_random_min_x_;
          x_max = left_random_max_x_;
        } else {
          if ((robot_x_limit_ - robot_map_pose.pose.position.x) >= 0) {
            x_min = left_random_min_x_;
            x_max = left_random_max_x_;
          } else {
            x_min = right_random_min_x_;
            x_max = right_random_max_x_;
          }
        }

        // Random goal generation in the escape region
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> x_dist(x_min, x_max);
        std::uniform_real_distribution<float> y_dist(0.0f, kFieldYMax);

        unsigned int enemy_cell_x, enemy_cell_y;
        if (!blackboard_->GetCostMap2D()->World2Map(
                enemy.pose.position.x, enemy.pose.position.y,
                enemy_cell_x, enemy_cell_y)) {
          chassis_executor_->Execute(whirl_vel_);
          return;
        }

        // Find a goal with sufficient obstacle cover
        float goal_x, goal_y;
        unsigned int goal_cell_x, goal_cell_y;
        int attempts = 0;

        while (attempts < kMaxSearchAttempts) {
          ++attempts;
          goal_x = x_dist(gen);
          goal_y = y_dist(gen);

          if (!blackboard_->GetCostMap2D()->World2Map(
                  goal_x, goal_y, goal_cell_x, goal_cell_y)) {
            continue;
          }

          auto index = blackboard_->GetCostMap2D()->GetIndex(goal_cell_x, goal_cell_y);
          if (blackboard_->GetCharMap()[index] >= kObstacleCostThreshold) {
            continue;
          }

          // Count obstacles along line of sight to enemy
          unsigned int obstacle_count = 0;
          for (FastLineIterator line(goal_cell_x, goal_cell_y, enemy_cell_x, enemy_cell_y);
               line.IsValid(); line.Advance()) {
            auto point_cost = blackboard_->GetCostMap2D()->GetCost(
                static_cast<unsigned int>(line.GetX()),
                static_cast<unsigned int>(line.GetY()));

            if (point_cost > kObstacleCostThreshold) {
              obstacle_count++;
            }
          }

          if (obstacle_count > kMinObstacleCount) {
            break;
          }
        }

        if (attempts >= kMaxSearchAttempts) {
          ROS_WARN("EscapeBehavior: failed to find covered escape goal after %d attempts",
                   kMaxSearchAttempts);
          chassis_executor_->Execute(whirl_vel_);
          return;
        }

        // Face toward the enemy while escaping
        Eigen::Vector2d pose_to_enemy(
            enemy.pose.position.x - robot_map_pose.pose.position.x,
            enemy.pose.position.y - robot_map_pose.pose.position.y);
        float goal_yaw = static_cast<float>(
            std::atan2(pose_to_enemy.coeffRef(1), pose_to_enemy.coeffRef(0)));
        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, goal_yaw);

        geometry_msgs::PoseStamped escape_goal;
        escape_goal.header.frame_id = "map";
        escape_goal.header.stamp = ros::Time::now();
        escape_goal.pose.position.x = goal_x;
        escape_goal.pose.position.y = goal_y;
        escape_goal.pose.orientation = quaternion;
        chassis_executor_->Execute(escape_goal);
      } else {
        // No enemy detected — spin in place
        chassis_executor_->Execute(whirl_vel_);
      }
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    left_x_limit_ = decision_config.escape().left_x_limit();
    right_x_limit_ = decision_config.escape().right_x_limit();
    robot_x_limit_ = decision_config.escape().robot_x_limit();
    left_random_min_x_ = decision_config.escape().left_random_min_x();
    left_random_max_x_ = decision_config.escape().left_random_max_x();
    right_random_min_x_ = decision_config.escape().right_random_min_x();
    right_random_max_x_ = decision_config.escape().right_random_max_x();

    whirl_vel_.twist.angular.z = decision_config.whirl_vel().angle_z_vel();
    whirl_vel_.twist.angular.y = decision_config.whirl_vel().angle_y_vel();
    whirl_vel_.twist.angular.x = decision_config.whirl_vel().angle_x_vel();

    return true;
  }

  ~EscapeBehavior() = default;

 private:
  // Configuration constants
  static constexpr unsigned char kObstacleCostThreshold = 253;
  static constexpr unsigned int kMinObstacleCount = 5;
  static constexpr float kFieldYMax = 5.0f;
  static constexpr int kMaxSearchAttempts = 1000;  // prevent infinite loop

  //! executor
  ChassisExecutor* const chassis_executor_;

  //! field limits from config
  float left_x_limit_ = 0, right_x_limit_ = 0;
  float robot_x_limit_ = 0;
  float left_random_min_x_ = 0, left_random_max_x_ = 0;
  float right_random_min_x_ = 0, right_random_max_x_ = 0;

  //! perception information
  Blackboard* const blackboard_;

  //! whirl velocity
  roborts_msgs::TwistAccel whirl_vel_;
};

} // namespace roborts_decision

#endif // ROBORTS_DECISION_ESCAPEBEHAVIOR_H
