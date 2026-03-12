#ifndef ROBORTS_DECISION_SEARCH_BEHAVIOR_H
#define ROBORTS_DECISION_SEARCH_BEHAVIOR_H

#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {

/**
 * @brief Search behavior: systematically search the area where an enemy was last seen.
 *
 * Divides the field into 4 quadrants and selects the appropriate search region
 * based on the enemy's last known position. Searches through waypoints in
 * the selected region, starting from the closest one.
 */
class SearchBehavior {
 public:
  SearchBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path)
      : chassis_executor_(chassis_executor),
        blackboard_(blackboard),
        search_index_(0),
        search_count_(0) {

    last_position_.header.frame_id = "map";
    last_position_.pose.orientation.x = 0;
    last_position_.pose.orientation.y = 0;
    last_position_.pose.orientation.z = 0;
    last_position_.pose.orientation.w = 1;

    last_position_.pose.position.x = 0;
    last_position_.pose.position.y = 0;
    last_position_.pose.position.z = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s: failed to load config file", __FUNCTION__);
    }
  }

  void Run() {
    auto executor_state = Update();

    if (executor_state != BehaviorState::RUNNING) {
      auto robot_map_pose = blackboard_->GetRobotMapPose();

      if (search_count_ == kInitialSearchCount) {
        // First search step: determine which quadrant and find closest waypoint
        const double x_diff = last_position_.pose.position.x - robot_map_pose.pose.position.x;
        const double y_diff = last_position_.pose.position.y - robot_map_pose.pose.position.y;
        const double last_x = last_position_.pose.position.x;
        const double last_y = last_position_.pose.position.y;

        // Select the search region based on quadrant
        if (last_x < kFieldMidX && last_y < kFieldMidY) {
          search_region_ = search_region_1_;
        } else if (last_x > kFieldMidX && last_y < kFieldMidY) {
          search_region_ = search_region_2_;
        } else if (last_x < kFieldMidX && last_y > kFieldMidY) {
          search_region_ = search_region_3_;
        } else {
          search_region_ = search_region_4_;
        }

        // Find the closest waypoint in the selected region
        double search_min_dist = std::numeric_limits<double>::max();
        for (unsigned int i = 0; i < search_region_.size(); ++i) {
          auto dist_sq = std::pow(search_region_[i].pose.position.x - last_x, 2)
                       + std::pow(search_region_[i].pose.position.y - last_y, 2);

          if (dist_sq < search_min_dist) {
            search_min_dist = dist_sq;
            search_index_ = i;
          }
        }

        // Navigate to last known enemy position first
        const double yaw = std::atan2(y_diff, x_diff);
        auto orientation = tf::createQuaternionMsgFromYaw(yaw);

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position = last_position_.pose.position;
        goal.pose.orientation = orientation;
        chassis_executor_->Execute(goal);
        search_count_--;

      } else if (search_count_ > 0) {
        // Subsequent steps: cycle through waypoints in the search region
        if (search_region_.empty()) {
          ROS_WARN("Search region is empty, cannot search");
          return;
        }
        auto search_goal = search_region_[search_index_++];
        chassis_executor_->Execute(search_goal);
        search_index_ = search_index_ % static_cast<unsigned int>(search_region_.size());
        search_count_--;
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

    // Load all 4 search regions from config
    LoadSearchRegion(decision_config.search_region_1(), search_region_1_);
    LoadSearchRegion(decision_config.search_region_2(), search_region_2_);
    LoadSearchRegion(decision_config.search_region_3(), search_region_3_);
    LoadSearchRegion(decision_config.search_region_4(), search_region_4_);

    ROS_INFO("Loaded search regions: R1=%zu, R2=%zu, R3=%zu, R4=%zu",
             search_region_1_.size(), search_region_2_.size(),
             search_region_3_.size(), search_region_4_.size());
    return true;
  }

  void SetLastPosition(geometry_msgs::PoseStamped last_position) {
    last_position_ = last_position;
    search_count_ = kInitialSearchCount;
  }

  ~SearchBehavior() = default;

 private:
  // Configuration constants
  static constexpr int kInitialSearchCount = 5;
  static constexpr double kFieldMidX = 4.2;
  static constexpr double kFieldMidY = 2.75;

  /**
   * @brief Helper to load a search region from protobuf config
   */
  template<typename RegionConfig>
  void LoadSearchRegion(const RegionConfig& config,
                        std::vector<geometry_msgs::PoseStamped>& region) {
    region.clear();
    region.reserve(config.size());
    for (int i = 0; i < config.size(); i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = config.Get(i).x();
      search_point.pose.position.y = config.Get(i).y();
      search_point.pose.position.z = config.Get(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(
          config.Get(i).roll(), config.Get(i).pitch(), config.Get(i).yaw());
      search_point.pose.orientation = quaternion;
      region.push_back(search_point);
    }
  }

  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! last known enemy position
  geometry_msgs::PoseStamped last_position_;

  //! search buffers
  std::vector<geometry_msgs::PoseStamped> search_region_1_;
  std::vector<geometry_msgs::PoseStamped> search_region_2_;
  std::vector<geometry_msgs::PoseStamped> search_region_3_;
  std::vector<geometry_msgs::PoseStamped> search_region_4_;
  std::vector<geometry_msgs::PoseStamped> search_region_;
  unsigned int search_count_;
  unsigned int search_index_;
};

} // namespace roborts_decision

#endif // ROBORTS_DECISION_SEARCH_BEHAVIOR_H
