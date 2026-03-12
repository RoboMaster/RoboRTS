#ifndef ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
#define ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H

#include <cmath>
#include <string>

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {

/**
 * @brief Behavior to navigate the robot back to its boot (starting) area.
 *
 * Reads the start position from config and drives the robot there.
 * Only initiates movement if the robot is sufficiently far from the boot
 * position or misaligned beyond the threshold.
 */
class BackBootAreaBehavior {
 public:
  BackBootAreaBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path)
      : chassis_executor_(chassis_executor),
        blackboard_(blackboard) {

    boot_position_.header.frame_id = "map";
    boot_position_.pose.orientation.x = 0;
    boot_position_.pose.orientation.y = 0;
    boot_position_.pose.orientation.z = 0;
    boot_position_.pose.orientation.w = 1;

    boot_position_.pose.position.x = 0;
    boot_position_.pose.position.y = 0;
    boot_position_.pose.position.z = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s: failed to load config file", __FUNCTION__);
    }
  }

  void Run() {
    auto executor_state = Update();

    if (executor_state != BehaviorState::RUNNING) {
      auto robot_map_pose = blackboard_->GetRobotMapPose();
      const double dx = boot_position_.pose.position.x - robot_map_pose.pose.position.x;
      const double dy = boot_position_.pose.position.y - robot_map_pose.pose.position.y;
      const double distance = std::sqrt(dx * dx + dy * dy);

      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(boot_position_.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      const double d_yaw = rot1.angleShortestPath(rot2);

      if (distance > kDistanceThreshold || d_yaw > kYawThreshold) {
        chassis_executor_->Execute(boot_position_);
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

    boot_position_.header.frame_id = "map";
    boot_position_.pose.position.x = decision_config.master_bot().start_position().x();
    boot_position_.pose.position.y = decision_config.master_bot().start_position().y();
    boot_position_.pose.position.z = decision_config.master_bot().start_position().z();

    auto master_quaternion = tf::createQuaternionMsgFromRollPitchYaw(
        decision_config.master_bot().start_position().roll(),
        decision_config.master_bot().start_position().pitch(),
        decision_config.master_bot().start_position().yaw());
    boot_position_.pose.orientation = master_quaternion;

    ROS_INFO("Boot position loaded: (%.2f, %.2f)",
             boot_position_.pose.position.x,
             boot_position_.pose.position.y);
    return true;
  }

  ~BackBootAreaBehavior() = default;

 private:
  // Thresholds for triggering return-to-boot navigation
  static constexpr double kDistanceThreshold = 0.2;  // meters
  static constexpr double kYawThreshold = 0.5;       // radians

  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! boot position
  geometry_msgs::PoseStamped boot_position_;
};

} // namespace roborts_decision

#endif // ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
