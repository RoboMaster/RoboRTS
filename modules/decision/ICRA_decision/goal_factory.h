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

#ifndef MODULE_DECISION_ICRA_GOAL_FACORY_H
#define MODULE_DECISION_ICRA_GOAL_FACORY_H

#include <Eigen/Core>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>

#include "messages/GlobalPlannerAction.h"
#include "messages/LocalPlannerAction.h"
#include "messages/BotGoalAction.h"
#include "messages/GoalTask.h"

#include "common/io.h"

#include "modules/planning/local_planner/line_iterator.h"

#include "modules/decision/behavior_tree/blackboard.h"
#include "modules/decision/behavior_tree/behavior_node.h"
#include "modules/perception/map/costmap/costmap_interface.h"

namespace rrts {
namespace decision {

class GoalFactory {
 public:

  typedef rrts::perception::map::CostmapInterface CostMap;
  typedef std::shared_ptr<CostMap> CostmapPtr;
  typedef actionlib::SimpleActionClient<messages::LocalPlannerAction> LocalActionClient;
  typedef actionlib::SimpleActionClient<messages::GlobalPlannerAction> GlobalActionClient;
  typedef actionlib::SimpleActionClient<messages::BotGoalAction> BotGoalActionClient;
  typedef messages::GlobalPlannerFeedbackConstPtr GlobalFeedback;
  typedef messages::LocalPlannerResultConstPtr LocalResult;
  typedef messages::LocalPlannerGoal LocalGoal;
  typedef messages::GlobalPlannerGoal GlobalGoal;
  typedef rrts::planning::local_planner::FastLineIterator LineIter;
  typedef rrts::perception::map::Costmap2D CostMap2D;
  typedef std::shared_ptr<GoalFactory> GoalFactoryPtr;

  GoalFactory(const Blackboard::Ptr &blackboard_ptr, const std::string & proto_file_path) :
      blackboard_ptr_(blackboard_ptr),
      global_planner_actionlib_client_("global_planner_node_action", true),
      local_planner_actionlib_client_("local_planner_node_action", true) {
    self_check_done_ = false;
    lost_enemy_ = true;
    master_ = false;
    arrive_ = false;
    switch_mode_ = false;
    search_count_ = 0;
    buff_count_ = 0;
    enemy_count_ = 0;
    patrol_count_ = 0;
    search_index_ = 0;

    search_x_min_ = 0;
    search_x_max_ = 8;
    search_y_min_ = 0;
    search_y_max_ = 5;

    action_state_ = BehaviorState::IDLE;

    last_goal_.header.frame_id = "map";
    last_goal_.pose.position.x = 0;
    last_goal_.pose.position.y = 0;

    LOG_INFO << "load file";
    LoadParam(proto_file_path);
    enemy_buff_.resize(2);

    ros::NodeHandle goal_nh;
    angle_vel_pub_ = angle_vel_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
//    goal_pub_ = goal_nh.advertise<geometry_msgs::PoseStamped>("goal_pose", 1);

    LOG_INFO << "create send goal client";
    if (master_) {
      goal_pub_ = goal_nh.advertise<messages::GoalTask>("/master/goal_task", 2);
      LOG_INFO << "create master send goal publisher";
    } else {
      goal_pub_ = goal_nh.advertise<messages::GoalTask>("/wing/goal_task", 2);
      LOG_INFO << "create wing send goal publisher";
    }

    // wing_goal_client_ = goal_nh_.serviceClient<messages::GoalTask>("wing/goal_task");

    LOG_INFO << "create cost map";
    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             "/modules/perception/map/costmap/config/costmap_parameter_config_for_decision.prototxt");
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

//    memcpy(charmap_, costmap_ptr_->GetCostMap()->GetCharMap(), costmap_2d_->GetSizeXCell() * costmap_2d_->GetSizeYCell());

    global_planner_actionlib_client_.waitForServer();
    LOG_INFO<<"Global planer server start!";
    local_planner_actionlib_client_.waitForServer();
    LOG_INFO<<"Local planer server start!";
  }

  ~GoalFactory() = default;

  void EscapeGoal() {

    arrive_ = false;

    if (action_state_ != BehaviorState::RUNNING) {

      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);

      if (blackboard_ptr_->GetEnemyDetected()) {

        geometry_msgs::PoseStamped enemy;
        enemy = blackboard_ptr_->GetEnemy();
        float goal_yaw, goal_x, goal_y;
        unsigned int goal_cell_x, goal_cell_y;
        unsigned int enemy_cell_x, enemy_cell_y;

        std::random_device rd;
        std::mt19937 gen(rd());

        UpdateRobotPose();
        float x_min, x_max;
        if (enemy.pose.position.x < left_x_limit_) {
          //std::uniform_real_distribution<float> x_uni_dis(5.5, 8);
          x_min = right_random_min_x_;
          x_max = right_random_max_x_;
        } else if (enemy.pose.position.x > right_x_limit_) {
          //std::uniform_real_distribution<float> x_uni_dis(0, 2.5);
          x_min = left_random_min_x_;
          x_max = left_random_max_x_;
        } else {
          //x_min = ((4 - robot_map_pose_.pose.position.x) >= 0) ? 0 : 5.5;

          if ((robot_x_limit_ - robot_map_pose_.pose.position.x) >= 0) {
            x_min = left_random_min_x_;
            x_max = left_random_max_x_;
          } else {
            x_min = right_random_min_x_;
            x_max = right_random_max_x_;
          }
        }

        std::uniform_real_distribution<float> x_uni_dis(x_min, x_max);
        std::uniform_real_distribution<float> y_uni_dis(0, 5);
        //std::uniform_real_distribution<float> yaw_uni_dis(-M_PI, M_PI);

        auto get_enemy_cell = costmap_ptr_->GetCostMap()->World2Map(enemy.pose.position.x,
                                              enemy.pose.position.y,
                                              enemy_cell_x,
                                              enemy_cell_y);
        if (!get_enemy_cell) {
          angle_vel_pub_.publish(whirl_vel_);
          action_state_ = BehaviorState::SUCCESS;
          return;
        }


        while (true) {
          goal_x = x_uni_dis(gen);
          goal_y = y_uni_dis(gen);
          auto get_goal_cell = costmap_ptr_->GetCostMap()->World2Map(goal_x,
                                                                goal_y,
                                                                goal_cell_x,
                                                                goal_cell_y);

          if (!get_goal_cell) {
            continue;
          }

          auto index = costmap_2d_->GetIndex(goal_cell_x, goal_cell_y);
//          costmap_2d_->GetCost(goal_cell_x, goal_cell_y);
          if (charmap_[index] >= 253) {
            continue;
          }

          unsigned int obstacle_count = 0;
          for(LineIter line( goal_cell_x, goal_cell_y, enemy_cell_x, enemy_cell_y); line.IsValid(); line.Advance()) {
            auto point_cost = costmap_2d_->GetCost(line.GetX(), line.GetY()); //current point's cost

            if(point_cost > 253){
              obstacle_count++;
            }

          }

          if (obstacle_count > 5) { //TODO:  this should write in the proto file
            break;
          }
        }
        Eigen::Vector2d pose_to_enemy(enemy.pose.position.x - robot_map_pose_.pose.position.x,
                                      enemy.pose.position.y - robot_map_pose_.pose.position.y);
        goal_yaw = static_cast<float > (std::atan2(pose_to_enemy.coeffRef(1), pose_to_enemy.coeffRef(0)));
        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(0,0,goal_yaw);

        geometry_msgs::PoseStamped escape_goal;
        escape_goal.header.frame_id = "map";
        escape_goal.header.stamp = ros::Time::now();
        escape_goal.pose.position.x = goal_x;
        escape_goal.pose.position.y = goal_y;
        escape_goal.pose.orientation = quaternion;
        //return exploration_goal;
        SendGoal(escape_goal);
      } else {
        angle_vel_pub_.publish(whirl_vel_);
        action_state_ = BehaviorState::SUCCESS;
        return;
      }
    }
    UpdateActionState();
    
  }

  BehaviorState Whirl() {
    blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
    blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
    angle_vel_pub_.publish(whirl_vel_);
    return BehaviorState::RUNNING;
  }

  void CancelWhirl() {
    geometry_msgs::Twist zero_angle_vel;
    zero_angle_vel.linear.x = 0;
    zero_angle_vel.linear.y = 0;
    zero_angle_vel.angular.z = 0;
    angle_vel_pub_.publish(zero_angle_vel);

  }

  void PatrolGoal() {

    arrive_ = false;
    if (action_state_ != BehaviorState::RUNNING) {

      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

      if (patrol_goals_.empty()) {
        LOG_ERROR << "patrol goal is empty";
        return;
      }

      SendGoal(patrol_goals_[patrol_count_]);
      patrol_count_ = ++patrol_count_ % point_size_;

      SendGoalTask(patrol_goals_[patrol_count_]);
      patrol_count_ = ++patrol_count_ % point_size_;

    }

  }

  void ChaseGoal() {

    arrive_ = false;
    if (action_state_ != BehaviorState::RUNNING) {

      enemy_buff_[enemy_count_++ % 2] = blackboard_ptr_->GetEnemy();
      if (lost_enemy_ && enemy_count_ >= 1) {
        lost_enemy_ = false;
        search_count_ = 5;
      }
      enemy_count_ = enemy_count_ % 2;

      UpdateRobotPose();
      auto wing_bot_goal = blackboard_ptr_->GetEnemy();
      auto wing_bot_goal_x = wing_bot_goal.pose.position.x;
      auto wing_bot_goal_y = wing_bot_goal.pose.position.y;
      auto dx = enemy_buff_[(enemy_count_ + 2 - 1) % 2].pose.position.x - robot_map_pose_.pose.position.x;
      auto dy = enemy_buff_[(enemy_count_ + 2 - 1) % 2].pose.position.y - robot_map_pose_.pose.position.y;
      auto yaw = std::atan2(dy, dx);
//      auto wing_yaw = std::atan2(wing_dy, wing_dx);
      for (int i = 20; i < 340; i += 5) {
        auto theta = (i / 180.) * M_PI;
        auto x = 2 * cos(theta);
        auto y = 2 * sin(theta);

        auto world_x = wing_bot_goal_x + x * cos(yaw) - y * sin(yaw);
        auto world_y = wing_bot_goal_y + x * sin(yaw) + y * cos(yaw);

        unsigned int wing_cell_x;
        unsigned int wing_cell_y;

        auto get_wing_cell = costmap_2d_->World2Map(world_x,
                                                    world_y,
                                                    wing_cell_x,
                                                    wing_cell_y);
        if (!get_wing_cell) {
          continue;
        }

        auto index = costmap_2d_->GetIndex(wing_cell_x, wing_cell_x);

        LOG_INFO << "world_x: " << world_x;
        LOG_INFO << "world_y: " << world_y;
        LOG_INFO << "wing_cell_x: " << wing_cell_x;
        LOG_INFO << "wing_cell_y: " << wing_cell_y;

//        if (wing_cell_x >= costmap_2d_->GetSizeXCell()
//            || wing_cell_x < 0
//            || wing_cell_y >= costmap_2d_->GetSizeYCell()
//            || wing_cell_y < 0) {
//          continue;
//        }

        if (charmap_[index]< 253) {
          wing_bot_goal.pose.position.x = world_x;
          wing_bot_goal.pose.position.y = world_y;

          wing_bot_goal.pose.orientation = tf::createQuaternionMsgFromYaw(-theta + yaw);

          break;
        }


      }
      SendGoalTask(wing_bot_goal);
      //SendGoal(blackboard_ptr_->GetEnemy());


//      auto start_yaw =  tf::getYaw(start_position_.pose.orientation);
//      auto last_yaw = tf::getYaw(last_goal_.pose.orientation);

//      auto d_yaw = start_yaw - last_yaw;

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) >= 1.0 && std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2.0) {
        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
        blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);
        if (switch_mode_) {
          global_planner_actionlib_client_.cancelGoal();
          local_planner_actionlib_client_.cancelGoal();
          switch_mode_ = false;
        }

        action_state_ = BehaviorState::SUCCESS;
        return;
      } else {

        auto orientation = tf::createQuaternionMsgFromYaw(yaw);
        geometry_msgs::PoseStamped reduce_goal;
        reduce_goal.pose.orientation = robot_map_pose_.pose.orientation;

        reduce_goal.header.frame_id = "map";
        reduce_goal.header.stamp = ros::Time::now();
        reduce_goal.pose.position.x = enemy_buff_[(enemy_count_ + 2 - 1) % 2].pose.position.x - 1.2 * cosf(yaw);
        reduce_goal.pose.position.y = enemy_buff_[(enemy_count_ + 2 - 1) % 2].pose.position.y - 1.2 * sinf(yaw);
        auto enemy_x = reduce_goal.pose.position.x;
        auto enemy_y = reduce_goal.pose.position.y;
        reduce_goal.pose.position.z = 1;
        unsigned int goal_cell_x, goal_cell_y;
        auto get_enemy_cell = costmap_ptr_->GetCostMap()->World2Map(enemy_x,
                                                                    enemy_y,
                                                                    goal_cell_x,
                                                                    goal_cell_y);

        if (!get_enemy_cell) {
          return;
        }

        auto robot_x = robot_map_pose_.pose.position.x;
        auto robot_y = robot_map_pose_.pose.position.y;
        unsigned int robot_cell_x, robot_cell_y;
        double goal_x, goal_y;
        costmap_ptr_->GetCostMap()->World2Map(robot_x,
                                              robot_y,
                                              robot_cell_x,
                                              robot_cell_y);

        if (costmap_2d_->GetCost(goal_cell_x, goal_cell_y) >= 253) {

          bool find_goal = false;
          for(LineIter line( goal_cell_x, goal_cell_y, robot_cell_x, robot_cell_x); line.IsValid(); line.Advance()) {

            auto point_cost = costmap_2d_->GetCost(line.GetX(), line.GetY()); //current point's cost

            if(point_cost >= 253){
              continue;

            } else {
              find_goal = true;
              costmap_ptr_->GetCostMap()->Map2World(line.GetX(),
                                                    line.GetY(),
                                                    goal_x,
                                                    goal_y);

              reduce_goal.pose.position.x = goal_x;
              reduce_goal.pose.position.y = goal_y;
              break;
            }

          }
          if (find_goal) {
            switch_mode_ = true;
            blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
            blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
            SendGoal(reduce_goal);
          } else {
            blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
            blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);
            action_state_ = BehaviorState::SUCCESS;
            if (switch_mode_) {
              global_planner_actionlib_client_.cancelGoal();
              local_planner_actionlib_client_.cancelGoal();
              switch_mode_ = false;
            }
            return;
          }

        } else {
          switch_mode_ = true;
          blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
          blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
          SendGoal(reduce_goal);
        }
      }


//      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
//      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

    } 
    UpdateActionState();
    
  }

  void SearchGoal () {

    arrive_ = false;
    double yaw;
    double x_diff;
    double y_diff;

    if (action_state_ != BehaviorState::RUNNING) {
      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      if (search_count_ == 5) {
        geometry_msgs::PoseStamped enemy_dis_1 = enemy_buff_[(enemy_count_ + 2 - 1) % 2];
        x_diff = enemy_dis_1.pose.position.x - robot_map_pose_.pose.position.x;
        y_diff = enemy_dis_1.pose.position.y - robot_map_pose_.pose.position.y;

//        search_x_min_ = (enemy_dis_1.pose.position.x > search_x_limit_) ? 4 : 0;
//        search_x_max_ = (enemy_dis_1.pose.position.x > search_x_limit_) ? 8 : 4;
//        search_x_max_ = (search_y_max_>costmap_2d_->GetSizeXCell()) ?  costmap_2d_->GetSizeXCell() : search_x_max_;
//
//        search_y_min_ = (enemy_dis_1.pose.position.y > search_y_limit_) ? 2.5 : 0;
//        search_y_max_ = (enemy_dis_1.pose.position.y > search_y_limit_) ? 5 : 2.5;
//        search_y_max_ = (search_y_max_>costmap_2d_->GetSizeYCell()) ?  costmap_2d_->GetSizeYCell() : search_y_max_;

        auto enemy_x = enemy_dis_1.pose.position.x;
        auto enemy_y = enemy_dis_1.pose.position.y;

        if (enemy_x < 4.2 && enemy_y < 2.75) {
          search_region_ = search_region_1_;
        } else if (enemy_x > 4.2 && enemy_y < 2.75) {
          search_region_ = search_region_2_;
        } else if (enemy_x < 4.2 && enemy_y > 2.75) {
          search_region_ = search_region_3_;
        } else {
          search_region_ = search_region_4_;
        }

        double search_min_dist = 99999;
        for (int i = 0; i < search_region_.size(); ++i) {
          auto dist_sq = std::pow(search_region_[i].pose.position.x - enemy_x, 2)
              + std::pow(search_region_[i].pose.position.y - enemy_y, 2);
          if (dist_sq < search_min_dist) {
            search_min_dist = dist_sq;
            search_index_ = i;
          }
        }

        yaw = std::atan2(y_diff, x_diff);

        auto orientation = tf::createQuaternionMsgFromYaw(yaw);

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position = enemy_dis_1.pose.position;
        goal.pose.orientation = orientation;
        SendGoalTask(goal);
        SendGoal(goal);
        lost_enemy_ = true;
        search_count_--;
      } else if (search_count_ > 0) {
        //RandomSearch();
//        std::random_device rd;
//        std::mt19937 gen(rd());
//
//        std::uniform_real_distribution<float> x_uni_dis(search_x_min_, search_x_max_);
//        std::uniform_real_distribution<float> y_uni_dis(search_y_min_, search_y_max_);

//        float goal_x;
//        float goal_y;
//        unsigned int goal_cell_x;
//        unsigned int goal_cell_y;
//        while (true) {
//          goal_x = x_uni_dis(gen);
//          goal_y = y_uni_dis(gen);
//          costmap_ptr_->GetCostMap()->World2Map(goal_x,
//                                                goal_y,
//                                                goal_cell_x,
//                                                goal_cell_y);
//
//          x_diff = goal_x - robot_map_pose_.pose.position.x;
//          y_diff = goal_y - robot_map_pose_.pose.position.y;
//
//          yaw = std::atan2(y_diff, x_diff);
//
//          if (costmap_2d_->GetCost(goal_cell_x, goal_cell_y) >= 240 || std::abs(yaw) < 0.2) {
//            continue;
//          } else {
//            break;
//          }
//
//        }


//        auto orientation = tf::createQuaternionMsgFromYaw(yaw);
//
//        geometry_msgs::PoseStamped goal;
//        goal.header.frame_id = "map";
//        goal.header.stamp = ros::Time::now();
//        goal.pose.position.x = goal_x;
//        goal.pose.position.y = goal_y;
//        goal.pose.orientation = orientation;

        auto search_goal = search_region_[(search_index_++ )];
        SendGoal(search_goal);
        search_index_ = search_index_% search_region_.size();
        search_count_--;

        search_goal = search_region_[(search_index_++)];
        SendGoalTask(search_goal);
        search_index_ = search_index_% search_region_.size();
        search_count_--;

      }
    }


  }

  void BuffGoal() {

    arrive_ = false;
    if (action_state_ != BehaviorState::RUNNING) {

      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

      if (patrol_goals_.empty()) {
        LOG_ERROR << "patrol goal is empty";
        return;
      }
      SendGoal(buff_position_[buff_count_%buff_size_]);
      buff_count_ = ++buff_count_ % buff_size_;
      SendGoalTask(wing_bot_task_point_);
    }

  }

  void TurnTOWoundedArmor() {

    if (action_state_ != BehaviorState::RUNNING) {

      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

      double yaw;
      switch (blackboard_ptr_->GetArmorAttacked()){
        case ArmorAttacked::FRONT:
          break;
        case ArmorAttacked::LEFT:
          yaw = M_PI/2.;
          break;
        case ArmorAttacked::BACK:
          yaw = M_PI;
          break;
        case ArmorAttacked::RIGHT:
          yaw = -M_PI/2.;
          break;
        default:
          return;
      }

      geometry_msgs::PoseStamped hurt_pose;
      auto quaternion = tf::createQuaternionMsgFromYaw(yaw);
      hurt_pose.header.frame_id="base_link";
      hurt_pose.header.stamp=ros::Time::now();
      hurt_pose.pose.orientation=quaternion;
      SendGoal(hurt_pose);
    }

  }

  void TurnToDetectedDirection() {

    if (action_state_ != BehaviorState::RUNNING) {

      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

      double yaw;
      switch (blackboard_ptr_->GetColordetected()) {
        case ColorDetected ::FRONT:
          break;
        case ColorDetected::LEFT:
          yaw = M_PI/2.;
          break;
        case ColorDetected::BACK:
          yaw = M_PI;
          break;
        case ColorDetected::RIGHT:
          yaw = -M_PI/2.;
          break;
        default:
          return;
      }

      geometry_msgs::PoseStamped hurt_pose;
      auto quaternion = tf::createQuaternionMsgFromYaw(yaw);
      hurt_pose.header.frame_id="base_link";
      hurt_pose.header.stamp=ros::Time::now();
      hurt_pose.pose.orientation=quaternion;
      SendGoal(hurt_pose);
    }


  }

  void GoAuxiliaryPosition() {

    if (action_state_ != BehaviorState::RUNNING) {

      if (blackboard_ptr_->GetGameProcess() != GameProcess::FIGHT) {

        if (blackboard_ptr_->GetArrive()) {

          UpdateRobotPose();
          auto dx = wing_bot_position_.pose.position.x - robot_map_pose_.pose.position.x;
          auto dy = wing_bot_position_.pose.position.y - robot_map_pose_.pose.position.y;

          // auto start_yaw = tf::getYaw(wing_bot_position_.pose.orientation);
          // auto last_yaw = tf::getYaw(robot_map_pose_.pose.orientation);

          tf::Quaternion rot1, rot2;
          tf::quaternionMsgToTF(wing_bot_position_.pose.orientation, rot1);
          tf::quaternionMsgToTF(robot_map_pose_.pose.orientation, rot2);
          auto d_yaw =  rot1.angleShortestPath(rot2);

          if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5) {
            blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
            blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
            SendGoal(wing_bot_position_);
          }

        }

        if (action_state_ == BehaviorState::SUCCESS) {
          blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELAX);
          blackboard_ptr_->ResetAllStatus();
          if(blackboard_ptr_->GetGameProcess() == GameProcess::NOT_START){
            self_check_done_ = false;
          }

          if(blackboard_ptr_->GetGameProcess() == GameProcess::PREPARATION && !self_check_done_){
            blackboard_ptr_->StartSelfCheck();
            self_check_done_ = true;
          }
        }

        return;

      } else if (blackboard_ptr_->GetEnemyDetected()) {
        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);

      } else {
        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);

      }

      //blackboard_ptr_->SetArrive(false);

      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

      SendGoal(blackboard_ptr_->GetAuxiliaryPosition());
    }

  }

  void SendGoalTask(geometry_msgs::PoseStamped goal) {

    auto dx = goal.pose.position.x - last_goal_.pose.position.x;
    auto dy = goal.pose.position.y - last_goal_.pose.position.y;

    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(goal.pose.orientation, rot1);
    tf::quaternionMsgToTF(last_goal_.pose.orientation, rot2);
    auto d_yaw =  rot1.angleShortestPath(rot2);

    if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) < 0.8) {
      return;
    }
    messages::GoalTask goal_task_msg;
    goal_task_msg.goal = goal;
    goal_task_msg.arrive = arrive_;

    goal_pub_.publish(goal_task_msg);

    last_goal_ = goal;

  }

  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();

    robot_tf_pose.frame_id_ = "base_link";
    robot_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    }
    catch (tf::LookupException &ex) {
      LOG_ERROR << "Transform Error looking up robot pose: " << ex.what();
    }
  }

  void SendGoal(geometry_msgs::PoseStamped goal) {

    global_planner_goal_.goal = goal;

    global_planner_actionlib_client_.sendGoal(global_planner_goal_,
                                              GlobalActionClient::SimpleDoneCallback(),
                                              GlobalActionClient::SimpleActiveCallback(),
                                              boost::bind(&GoalFactory::GlobalPlannerFeedbackCallback, this, _1));
  }

  void GlobalPlannerFeedbackCallback(const GlobalFeedback& feedback){
    if (!feedback->path.poses.empty()) {
      local_planner_goal_.route = feedback->path;
      local_planner_actionlib_client_.sendGoal(local_planner_goal_);
    }
  }

  void UpdateActionState(){
    auto state = global_planner_actionlib_client_.getState();
    if (state == actionlib::SimpleClientGoalState::ACTIVE){
      LOG_INFO << " "<<__FUNCTION__<< ": ACTIVE";
      action_state_ = BehaviorState::RUNNING;

    } else if (state == actionlib::SimpleClientGoalState::PENDING) {
      LOG_INFO << " "<<__FUNCTION__<< ": PENDING";
      action_state_ = BehaviorState::RUNNING;

    } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      action_state_ = BehaviorState::SUCCESS;

    } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
      action_state_ = BehaviorState::FAILURE;

    } else {
      LOG_INFO<<"Error: "<<state.toString();
      action_state_ = BehaviorState::FAILURE;
    }
  }

  BehaviorState GetActionState() {
    return action_state_;
  }

  void CancelGoal() {
    LOG_INFO<<"Cancel Goal!";
    switch_mode_ = false;
    global_planner_actionlib_client_.cancelGoal();
    local_planner_actionlib_client_.cancelGoal();
    action_state_ = BehaviorState::IDLE;
    blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

  }
  void CancelSearch() {
    search_count_ = 0;
  }

  void BackBootArea() {

    UpdateRobotPose();
    if (action_state_ != BehaviorState::RUNNING) {
      auto dx = master_start_position_.pose.position.x - robot_map_pose_.pose.position.x;
      auto dy = master_start_position_.pose.position.y - robot_map_pose_.pose.position.y;

      auto start_yaw = tf::getYaw(master_start_position_.pose.orientation);
      auto last_yaw = tf::getYaw(robot_map_pose_.pose.orientation);

      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(master_start_position_.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose_.pose.orientation, rot2);
      auto d_yaw =  rot1.angleShortestPath(rot2);

      //auto d_yaw = std::abs(start_yaw - last_yaw);
//      LOG_INFO << "robot qua: " << robot_map_pose_.pose.orientation;
//      LOG_INFO << "master qua: " << master_start_position_.pose.orientation;
//      LOG_INFO << "rot1 qua: " << rot1;
//      LOG_INFO << "rot2 qua: " << rot2;

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5) {
//        LOG_INFO << "dist: " << std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
//        LOG_INFO << "yaw: " << d_yaw;
        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
        blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
        SendGoal(master_start_position_);
        //arrive_ = false;
        //SendGoalTask(wing_bot_position_);
        //last_goal_ = start_position_;
      }

      if (action_state_ == BehaviorState::SUCCESS) {
        arrive_ = true;
        SendGoalTask(wing_bot_position_);
        if(blackboard_ptr_->GetGameProcess() == GameProcess::NOT_START){
          self_check_done_ = false; 
        }
        if(blackboard_ptr_->GetGameProcess() == GameProcess::PREPARATION && !self_check_done_){
          blackboard_ptr_->StartSelfCheck();
          self_check_done_ = true;
        }
      }

    }

  }

  void LoadParam(const std::string &proto_file_path) {
    rrts::decision::DecisionConfig decision_config;
    rrts::common::ReadProtoFromTextFile(proto_file_path, &decision_config);
    //patrol goal config
    point_size_ = decision_config.point().size();
    patrol_goals_.resize(point_size_);
    for (int i = 0; i != point_size_; i++) {
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

    buff_size_ = decision_config.buff_point().size();
    buff_position_.resize(buff_size_);
    for (int i = 0; i != buff_size_; i++) {
      buff_position_[i].header.frame_id = "map";
      buff_position_[i].pose.position.x = decision_config.buff_point(i).x();
      buff_position_[i].pose.position.y = decision_config.buff_point(i).y();
      buff_position_[i].pose.position.z = decision_config.buff_point(i).z();

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.buff_point(i).roll(),
                                                              decision_config.buff_point(i).pitch(),
                                                              decision_config.buff_point(i).yaw());
      buff_position_[i].pose.orientation.x = quaternion.x();
      buff_position_[i].pose.orientation.y = quaternion.y();
      buff_position_[i].pose.orientation.z = quaternion.z();
      buff_position_[i].pose.orientation.w = quaternion.w();
    }

    search_region_.resize(decision_config.search_region_1().size());
    for (int i = 0; i != decision_config.search_region_1().size(); i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_1(i).x();
      search_point.pose.position.y = decision_config.search_region_1(i).y();
      search_point.pose.position.z = decision_config.search_region_1(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_1(i).roll(),
                                                                decision_config.search_region_1(i).pitch(),
                                                                decision_config.search_region_1(i).yaw());
      search_point.pose.orientation = quaternion;
      search_region_1_.push_back(search_point);
    }

    for (int i = 0; i != decision_config.search_region_2().size(); i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_2(i).x();
      search_point.pose.position.y = decision_config.search_region_2(i).y();
      search_point.pose.position.z = decision_config.search_region_2(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_2(i).roll(),
                                                                decision_config.search_region_2(i).pitch(),
                                                                decision_config.search_region_2(i).yaw());
      search_point.pose.orientation = quaternion;
      search_region_2_.push_back(search_point);
    }

    for (int i = 0; i != decision_config.search_region_3().size(); i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_3(i).x();
      search_point.pose.position.y = decision_config.search_region_3(i).y();
      search_point.pose.position.z = decision_config.search_region_3(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_3(i).roll(),
                                                                decision_config.search_region_3(i).pitch(),
                                                                decision_config.search_region_3(i).yaw());
      search_point.pose.orientation = quaternion;
      search_region_3_.push_back(search_point);
    }

    for (int i = 0; i != decision_config.search_region_4().size(); i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_4(i).x();
      search_point.pose.position.y = decision_config.search_region_4(i).y();
      search_point.pose.position.z = decision_config.search_region_4(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_4(i).roll(),
                                                                decision_config.search_region_4(i).pitch(),
                                                                decision_config.search_region_4(i).yaw());
      search_point.pose.orientation = quaternion;
      search_region_4_.push_back(search_point);
    }

    left_x_limit_ = decision_config.escape().left_x_limit();
    right_x_limit_ = decision_config.escape().right_x_limit();
    robot_x_limit_ = decision_config.escape().robot_x_limit();
    left_random_min_x_ = decision_config.escape().left_random_min_x();
    left_random_max_x_ = decision_config.escape().left_random_max_x();
    right_random_min_x_ = decision_config.escape().right_random_min_x();
    right_random_max_x_ = decision_config.escape().right_random_max_x();
    search_x_limit_ = decision_config.search_limit().x_limit();
    search_y_limit_ = decision_config.search_limit().y_limit();

    whirl_vel_.angular.z = decision_config.whirl_vel().angle_z_vel();
    whirl_vel_.angular.y = decision_config.whirl_vel().angle_y_vel();
    whirl_vel_.angular.x = decision_config.whirl_vel().angle_x_vel();


    master_start_position_.header.frame_id = "map";

    master_start_position_.pose.position.x = decision_config.master_bot().start_position().x();
    master_start_position_.pose.position.z = decision_config.master_bot().start_position().z();
    master_start_position_.pose.position.y = decision_config.master_bot().start_position().y();

    auto master_quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.master_bot().start_position().roll(),
                                                                     decision_config.master_bot().start_position().pitch(),
                                                                     decision_config.master_bot().start_position().yaw());
    master_start_position_.pose.orientation = master_quaternion;
    //LOG_INFO << "master qua: " << master_quaternion;



    wing_bot_position_.header.frame_id = "map";

    wing_bot_position_.pose.position.x = decision_config.wing_bot().start_position().x();
    wing_bot_position_.pose.position.z = decision_config.wing_bot().start_position().z();
    wing_bot_position_.pose.position.y = decision_config.wing_bot().start_position().y();

    auto wing_bot_quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.wing_bot().start_position().roll(),
                                                                       decision_config.wing_bot().start_position().pitch(),
                                                                       decision_config.wing_bot().start_position().yaw());
    wing_bot_position_.pose.orientation = wing_bot_quaternion;


    wing_bot_task_point_.header.frame_id = "map";

    wing_bot_task_point_.pose.position.x = decision_config.wing_bot_task_point().x();
    wing_bot_task_point_.pose.position.z = decision_config.wing_bot_task_point().z();
    wing_bot_task_point_.pose.position.y = decision_config.wing_bot_task_point().y();

    auto wing_bot_task_quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.wing_bot_task_point().roll(),
                                                                            decision_config.wing_bot_task_point().pitch(),
                                                                            decision_config.wing_bot_task_point().yaw());

    wing_bot_task_point_.pose.orientation = wing_bot_task_quaternion;

    master_ = decision_config.master();

  }

  void RandomSearch() {
    if (!lost_enemy_) {
      UpdateRobotPose();
      geometry_msgs::PoseStamped enemy_dis_1 = enemy_buff_[(enemy_count_ + 2 - 1) % 2];
//      geometry_msgs::PoseStamped enemy_dis_2 = enemy_buff_[(enemy_count_ + 2 - 2) % 2];
//      double time_diff = (enemy_dis_1.header.stamp - enemy_dis_2.header.stamp).toSec();
//      double x_diff = enemy_dis_1.pose.position.x - enemy_dis_2.pose.position.x;
//      double y_diff = enemy_dis_1.pose.position.y - enemy_dis_2.pose.position.y;

//      double x_diff = enemy_dis_1.pose.position.x - robot_map_pose_.pose.position.x;
//      double y_diff = enemy_dis_1.pose.position.y - robot_map_pose_.pose.position.y;
//
//      search_x_min_ = (enemy_dis_1.pose.position.x > search_x_limit_) ? 4 : 0;
//      search_x_max_ = (enemy_dis_1.pose.position.x > search_x_limit_) ? 8 : 4;
//      search_x_max_ = (search_y_max_>costmap_2d_->GetSizeXCell()) ?  costmap_2d_->GetSizeXCell() : search_x_max_;
//
//      search_y_min_ = (enemy_dis_1.pose.position.y > search_y_limit_) ? 2.5 : 0;
//      search_y_max_ = (enemy_dis_1.pose.position.y > search_y_limit_) ? 5 : 2.5;
//      search_y_max_ = (search_y_max_>costmap_2d_->GetSizeYCell()) ?  costmap_2d_->GetSizeYCell() : search_y_max_;

//      search_y_min_ = enemy_dis_1.pose.position.y;
//
//      search_y_max_ = enemy_dis_1.pose.position.y + y_diff * 5;

//      yaw_ = std::atan2(y_diff, x_diff);
//      lost_enemy_ = true;
    }
  }

  bool SearchValid () {
    return search_count_ > 0;
  }
 protected:
  Blackboard::Ptr blackboard_ptr_;
  CostmapPtr costmap_ptr_;
  CostMap2D* costmap_2d_;
  std::shared_ptr<tf::TransformListener> tf_ptr_;
  unsigned char * charmap_;
  LocalActionClient local_planner_actionlib_client_;
  GlobalActionClient global_planner_actionlib_client_;

  GlobalGoal global_planner_goal_;
  LocalGoal local_planner_goal_;

  geometry_msgs::PoseStamped robot_map_pose_;
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  float left_x_limit_, right_x_limit_;
  float robot_x_limit_;
  float left_random_min_x_, left_random_max_x_;
  float right_random_min_x_, right_random_max_x_;
  unsigned int patrol_count_;
  unsigned int point_size_;
  std::vector<geometry_msgs::PoseStamped> enemy_buff_;
  unsigned int enemy_count_;
  std::vector<geometry_msgs::PoseStamped> buff_position_;
  unsigned int buff_count_;
  unsigned int buff_size_;
  bool lost_enemy_;
  unsigned int search_count_;
  geometry_msgs::PoseStamped last_buff_goal_;
  ros::NodeHandle angle_vel_nh_;
  ros::Publisher angle_vel_pub_;

  ros::ServiceClient goal_client_;
  //ros::ServiceClient wing_goal_client_;
  ros::Publisher goal_pub_;
  bool master_;
  bool arrive_;

  geometry_msgs::Twist whirl_vel_;

  geometry_msgs::PoseStamped last_goal_;

  BehaviorState action_state_;


  float search_x_limit_;
  float search_y_limit_;

  double search_x_min_;
  double search_x_max_;
  double search_y_min_;
  double search_y_max_;
  double yaw_;

  geometry_msgs::PoseStamped master_start_position_;
  geometry_msgs::PoseStamped wing_bot_position_;
  geometry_msgs::PoseStamped wing_bot_task_point_;

  std::vector<geometry_msgs::PoseStamped> search_region_1_;
  std::vector<geometry_msgs::PoseStamped> search_region_2_;
  std::vector<geometry_msgs::PoseStamped> search_region_3_;
  std::vector<geometry_msgs::PoseStamped> search_region_4_;

  std::vector<geometry_msgs::PoseStamped> search_region_;
  unsigned int search_index_;

  bool switch_mode_;
  bool self_check_done_;
};

} // namespace rrts
} // namespace decision
#endif //MODULE_DECISION_ICRA_GOAL_FACORY_H
