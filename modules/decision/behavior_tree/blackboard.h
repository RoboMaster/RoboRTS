/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H
#define MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <ros/ros.h>

#include "messages/ArmorDetectionAction.h"
#include "messages/LocalizationAction.h"

#include "common/io.h"
#include "modules/decision/proto/decision.pb.h"


namespace rrts{
namespace decision {

class Blackboard {
 public:

  typedef std::shared_ptr<Blackboard> Ptr;
  Blackboard():
      localization_actionlib_client_("localization_node_action", true),
      armor_detection_actionlib_client_("armor_detection_node_action", true),
      enemy_found_(false),
      has_goal_(false),
      patrol_(true)
  {
    LoadParam();
    ros::NodeHandle rviz_nh("move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1,
                                                              &Blackboard::EnemyCallback, this);
    // Connect to localization server and start
    localization_actionlib_client_.waitForServer();
    LOG_INFO << "Localization module has been connected!";
    localization_goal_.command = 1;
    localization_actionlib_client_.sendGoal(localization_goal_);

  };
  ~Blackboard() {};

  void EnemyCallback(const geometry_msgs::PoseStamped::ConstPtr & enemy){
    SetEnemy(*enemy);
  }
  bool HasGoal(){
    return has_goal_;
  }
  void SetGoal(const geometry_msgs::PoseStamped & goal) {
    goal_ = goal;
    has_goal_=true;
  }
  geometry_msgs::PoseStamped GetGoal() {
    has_goal_ = false;
    return goal_;
  }

  bool HasEnemy() {
    return enemy_found_;
  }
  void SetEnemy(const geometry_msgs::PoseStamped &enemy) {
    enemy_ = enemy;
    enemy_found_ = true;
  }
  geometry_msgs::PoseStamped GetEnemy() {
    enemy_found_ = false;
    return enemy_;
  }

  geometry_msgs::PoseStamped GetPatrol() {
    patrol_goals_iter_++;
    if (patrol_goals_iter_ == patrol_goals_.end()) {
      patrol_goals_iter_ = patrol_goals_.begin();
    }
    return *patrol_goals_iter_;
  }


 private:
  void LoadParam() {
    rrts::decision::DecisionConfig decision_config;
    std::string file_name = "modules/decision/config/decision.prototxt";
    rrts::common::ReadProtoFromTextFile(file_name, &decision_config);

    //patrol goal config
    unsigned int point_size = decision_config.point().size();
    patrol_goals_.resize(point_size);
    for (int i = 0; i != point_size; i++) {
      patrol_goals_[i].header.frame_id = "map";
      patrol_goals_[i].pose.position.x = decision_config.point(i).x();
      patrol_goals_[i].pose.position.y = decision_config.point(i).y();
      patrol_goals_[i].pose.position.z = decision_config.point(i).z();

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.point(i).roll(),
                                                              decision_config.point(i).pitch(),
                                                              decision_config.point(i).yaw());
      patrol_goals_[i].pose.orientation.x = quaternion.x();
      patrol_goals_[i].pose.orientation.y = quaternion.y();
      patrol_goals_[i].pose.orientation.z = quaternion.z();
      patrol_goals_[i].pose.orientation.w = quaternion.w();
    }
    patrol_goals_iter_ = patrol_goals_.begin();
  }
  //!
  ros::Subscriber enemy_sub_;
  //! Action client
  actionlib::SimpleActionClient<messages::LocalizationAction> localization_actionlib_client_;
  actionlib::SimpleActionClient<messages::ArmorDetectionAction> armor_detection_actionlib_client_;

  //! Action goal
  messages::LocalizationGoal localization_goal_;
  messages::ArmorDetectionGoal armor_detection_goal_;

  //! Goal
  geometry_msgs::PoseStamped goal_;
  bool has_goal_;

  //! EnemyFound
  geometry_msgs::PoseStamped enemy_;
  bool enemy_found_;

  //! Patrol
  bool patrol_;
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  std::vector<geometry_msgs::PoseStamped>::iterator patrol_goals_iter_;

};
} //namespace decision
} //namespace rrts
#endif //MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H
