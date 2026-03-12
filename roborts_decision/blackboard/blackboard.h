/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <mutex>
#include <memory>
#include <string>
#include <geometry_msgs/PoseStamped.h>

#include "roborts_msgs/ArmorDetectionAction.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"

namespace roborts_decision {

/**
 * @brief Central blackboard for sharing perception and planning data between behaviors.
 *
 * Manages:
 * - Enemy detection via ArmorDetection actionlib feedback
 * - Goal commands from RViz
 * - Robot pose from tf
 * - Costmap access for path planning queries
 *
 * All accessors are thread-safe via mutex protection.
 */
class Blackboard {
 public:
  using Ptr = std::shared_ptr<Blackboard>;
  using CostMap = roborts_costmap::CostmapInterface;
  using CostMap2D = roborts_costmap::Costmap2D;

  explicit Blackboard(const std::string &proto_file_path)
      : enemy_detected_(false),
        new_goal_(false),
        armor_detection_actionlib_client_("armor_detection_node_action", true) {

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    std::string map_path = ros::package::getPath("roborts_costmap")
        + "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_, map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();
    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    // Subscribe to RViz goal for manual enemy position input (simulation)
    ros::NodeHandle rviz_nh("/move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>(
        "goal", 1, &Blackboard::GoalCallback, this);

    ros::NodeHandle nh;

    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

    if (!decision_config.simulate()) {
      ROS_INFO("Connecting to armor detection module...");
      armor_detection_actionlib_client_.waitForServer();
      ROS_INFO("Armor detection module connected.");

      armor_detection_goal_.command = 1;
      armor_detection_actionlib_client_.sendGoal(
          armor_detection_goal_,
          actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
          actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
          boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));
    } else {
      ROS_INFO("Running in simulation mode — armor detection disabled.");
    }
  }

  ~Blackboard() = default;

  // Non-copyable
  Blackboard(const Blackboard&) = delete;
  Blackboard& operator=(const Blackboard&) = delete;

  //--------------------------------------------------------------------------
  // Enemy Detection
  //--------------------------------------------------------------------------

  void ArmorDetectionFeedbackCallback(
      const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback) {
    if (feedback->detected) {
      ROS_INFO_THROTTLE(1.0, "Enemy detected!");

      tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
      camera_pose_msg = feedback->enemy_pos;

      const double yaw = std::atan2(camera_pose_msg.pose.position.y,
                                     camera_pose_msg.pose.position.x);

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);
      camera_pose_msg.pose.orientation.w = quaternion.w();
      camera_pose_msg.pose.orientation.x = quaternion.x();
      camera_pose_msg.pose.orientation.y = quaternion.y();
      camera_pose_msg.pose.orientation.z = quaternion.z();
      poseStampedMsgToTF(camera_pose_msg, tf_pose);

      tf_pose.stamp_ = ros::Time(0);
      try {
        tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
        tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);

        if (GetDistance(global_pose_msg, enemy_pose_) > kPositionChangeThreshold ||
            GetAngle(global_pose_msg, enemy_pose_) > kAngleChangeThreshold) {
          std::lock_guard<std::mutex> lock(enemy_mutex_);
          enemy_pose_ = global_pose_msg;
        }
      } catch (tf::TransformException &ex) {
        ROS_ERROR_THROTTLE(1.0, "TF error transforming enemy pose: %s", ex.what());
      }

      std::lock_guard<std::mutex> lock(enemy_mutex_);
      enemy_detected_ = true;
    } else {
      std::lock_guard<std::mutex> lock(enemy_mutex_);
      enemy_detected_ = false;
    }
  }

  geometry_msgs::PoseStamped GetEnemy() {
    std::lock_guard<std::mutex> lock(enemy_mutex_);
    return enemy_pose_;
  }

  bool IsEnemyDetected() {
    std::lock_guard<std::mutex> lock(enemy_mutex_);
    ROS_DEBUG("%s: %d", __FUNCTION__, static_cast<int>(enemy_detected_));
    return enemy_detected_;
  }

  //--------------------------------------------------------------------------
  // Goal Management
  //--------------------------------------------------------------------------

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    new_goal_ = true;
    goal_ = *goal;
  }

  geometry_msgs::PoseStamped GetGoal() {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    return goal_;
  }

  bool IsNewGoal() {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (new_goal_) {
      new_goal_ = false;
      return true;
    }
    return false;
  }

  //--------------------------------------------------------------------------
  // Utility Methods
  //--------------------------------------------------------------------------

  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                    const geometry_msgs::PoseStamped &pose2) const {
    const double dx = pose1.pose.position.x - pose2.pose.position.x;
    const double dy = pose1.pose.position.y - pose2.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2) const {
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(pose1.pose.orientation, rot1);
    tf::quaternionMsgToTF(pose2.pose.orientation, rot2);
    return rot1.angleShortestPath(rot2);
  }

  const geometry_msgs::PoseStamped GetRobotMapPose() {
    UpdateRobotPose();
    return robot_map_pose_;
  }

  const std::shared_ptr<CostMap> GetCostMap() {
    return costmap_ptr_;
  }

  const CostMap2D* GetCostMap2D() {
    return costmap_2d_;
  }

  const unsigned char* GetCharMap() {
    return charmap_;
  }

 private:
  // Configuration constants
  static constexpr double kPositionChangeThreshold = 0.2;
  static constexpr double kAngleChangeThreshold = 0.2;

  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();
    robot_tf_pose.frame_id_ = "base_link";
    robot_tf_pose.stamp_ = ros::Time();

    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    } catch (tf::LookupException &ex) {
      ROS_ERROR_THROTTLE(1.0, "Transform error getting robot pose: %s", ex.what());
    }
  }

  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! Thread synchronization
  mutable std::mutex enemy_mutex_;
  mutable std::mutex goal_mutex_;

  //! Enemy detection
  ros::Subscriber enemy_sub_;

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;

  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  geometry_msgs::PoseStamped enemy_pose_;
  bool enemy_detected_;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

  //! robot map pose
  geometry_msgs::PoseStamped robot_map_pose_;
};

} // namespace roborts_decision

#endif // ROBORTS_DECISION_BLACKBOARD_H
