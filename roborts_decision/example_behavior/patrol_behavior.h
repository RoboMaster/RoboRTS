#ifndef ROBORTS_DECISION_PATROL_BEHAVIOR_H
#define ROBORTS_DECISION_PATROL_BEHAVIOR_H

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
 * @brief Patrol behavior: cycle through a set of waypoints loaded from config.
 *
 * Navigates to patrol goals in round-robin order. When a goal is reached,
 * the next goal in the sequence is sent to the chassis executor.
 */
class PatrolBehavior {
 public:
  PatrolBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path)
      : chassis_executor_(chassis_executor),
        blackboard_(blackboard),
        patrol_count_(0),
        point_size_(0) {

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s: failed to load config file", __FUNCTION__);
    }
  }

  void Run() {
    auto executor_state = Update();
    ROS_DEBUG("Patrol state: %d", static_cast<int>(executor_state));

    if (executor_state != BehaviorState::RUNNING) {
      if (patrol_goals_.empty()) {
        ROS_ERROR("Patrol goals are empty — check config file");
        return;
      }

      ROS_DEBUG("Patrol: navigating to waypoint %d/%d",
                patrol_count_ + 1, point_size_);
      chassis_executor_->Execute(patrol_goals_[patrol_count_]);
      patrol_count_ = (patrol_count_ + 1) % point_size_;
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

    point_size_ = static_cast<unsigned int>(decision_config.point().size());
    patrol_goals_.resize(point_size_);

    for (unsigned int i = 0; i < point_size_; i++) {
      patrol_goals_[i].header.frame_id = "map";
      patrol_goals_[i].pose.position.x = decision_config.point(i).x();
      patrol_goals_[i].pose.position.y = decision_config.point(i).y();
      patrol_goals_[i].pose.position.z = decision_config.point(i).z();

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(
          decision_config.point(i).roll(),
          decision_config.point(i).pitch(),
          decision_config.point(i).yaw());
      patrol_goals_[i].pose.orientation.x = quaternion.x();
      patrol_goals_[i].pose.orientation.y = quaternion.y();
      patrol_goals_[i].pose.orientation.z = quaternion.z();
      patrol_goals_[i].pose.orientation.w = quaternion.w();
    }

    ROS_INFO("Loaded %d patrol waypoints", point_size_);
    return true;
  }

  ~PatrolBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! patrol buffer
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  unsigned int patrol_count_;
  unsigned int point_size_;
};

} // namespace roborts_decision

#endif // ROBORTS_DECISION_PATROL_BEHAVIOR_H
