//vic_decision
// Created by vic on 4/25/18.
//

#ifndef MODULE_DECISION_VIC_GOAL_FACORY_H
#define MODULE_DECISION_VIC_GOAL_FACORY_H

#include <Eigen/Core>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>

#include "messages/GlobalPlannerAction.h"
#include "messages/LocalPlannerAction.h"

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
    lost_enemy_ = true;
    search_count_ = 0;
    buff_count_ = 0;
    enemy_count_ = 0;
    patrol_count_ = 0;

    search_x_min_ = 0;
    search_x_max_ = 8;
    search_y_min_ = 0;
    search_y_max_ = 5;

    action_state_ = BehaviorState::IDLE;

    last_goal_.header.frame_id = "map";
    last_goal_.pose.position.x = 0;
    last_goal_.pose.position.y = 0;

    LoadParam(proto_file_path);
    enemy_buff_.resize(2);

    angle_vel_pub_ = angle_vel_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             "modules/perception/map/costmap/config/costmap_parameter_config_for_decision.prototxt");
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();
    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    global_planner_actionlib_client_.waitForServer();
    LOG_INFO<<"Global planer server start!";
    local_planner_actionlib_client_.waitForServer();
    LOG_INFO<<"Local planer server start!";
  }

  ~GoalFactory() = default;

  void EscapeGoal() {

    if (action_state_ != BehaviorState::RUNNING) {

      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);

      if (blackboard_ptr_->GetEnemyDetected()) {

        geometry_msgs::PoseStamped enemy;
        enemy = blackboard_ptr_->GetEnemy();
        float goal_yaw,goal_x,goal_y;
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

        costmap_ptr_->GetCostMap()->World2Map(enemy.pose.position.x,
                                              enemy.pose.position.y,
                                              enemy_cell_x,
                                              enemy_cell_y);

        while (true) {
          goal_x = x_uni_dis(gen);
          goal_y = y_uni_dis(gen);
          costmap_ptr_->GetCostMap()->World2Map(goal_x,
                                                goal_y,
                                                goal_cell_x,
                                                goal_cell_y);

          if (costmap_2d_->GetCost(goal_cell_x, goal_cell_y) >= 240) {
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
        goal_yaw = std::atan2(pose_to_enemy.coeffRef(1), pose_to_enemy.coeffRef(0));
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,goal_yaw);

        geometry_msgs::PoseStamped escape_goal;
        escape_goal.header.frame_id = "map";
        escape_goal.header.stamp = ros::Time::now();
        escape_goal.pose.position.x = goal_x;
        escape_goal.pose.position.y = goal_y;
        escape_goal.pose.orientation.w = quaternion.w();
        escape_goal.pose.orientation.x = quaternion.x();
        escape_goal.pose.orientation.y = quaternion.y();
        escape_goal.pose.orientation.z = quaternion.z();
        //return exploration_goal;
        SendGoal(escape_goal);
        action_state_ = BehaviorState::RUNNING;
      } else {
        angle_vel_pub_.publish(whirl_vel_);
      }
    }

  }

  BehaviorState Whirl() {
    blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
    blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
    angle_vel_pub_.publish(whirl_vel_);
    return BehaviorState::RUNNING;
  }

  void CancelWhirl() {
    geometry_msgs::Twist zero_angle_vel;
    zero_angle_vel.angular.x = 0;
    zero_angle_vel.angular.y = 0;
    zero_angle_vel.angular.z = 0;
    angle_vel_pub_.publish(zero_angle_vel);

  }

  void PatrolGoal() {

    if (action_state_ != BehaviorState::RUNNING) {

      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

      if (patrol_goals_.empty()) {
        LOG_ERROR << "patrol goal is empty";
        return;
      }

      SendGoal(patrol_goals_[patrol_count_%point_size_]);
      action_state_ = BehaviorState::RUNNING;
      patrol_count_ = ++patrol_count_ % point_size_;
    }

  }

  void ChaseGoal() {

    if (action_state_ != BehaviorState::RUNNING) {

      UpdateRobotPose();

      enemy_buff_[enemy_count_++ % 2] = blackboard_ptr_->GetEnemy();
      SendGoal(blackboard_ptr_->GetEnemy());
      action_state_ = BehaviorState::RUNNING;
      if (lost_enemy_ && enemy_count_ >= 1) {
        lost_enemy_ = false;
        search_count_ = 5;
      }
      enemy_count_ = enemy_count_ % 2;

      auto dx = enemy_buff_[(enemy_count_ + 2 - 1) % 2].pose.position.x - robot_map_pose_.pose.position.x;
      auto dy = enemy_buff_[(enemy_count_ + 2 - 1) % 2].pose.position.y - robot_map_pose_.pose.position.y;

//      auto start_yaw =  tf::getYaw(start_position_.pose.orientation);
//      auto last_yaw = tf::getYaw(last_goal_.pose.orientation);

//      auto d_yaw = start_yaw - last_yaw;

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) >= 1 && std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2.5) {
        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
        blackboard_ptr_->SetChassisMode(ChassisMode::DODGE_MODE);

      } else if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) < 1) {
        auto yaw = std::atan2(dy, dx);
        enemy_buff_[(enemy_count_ + 2 - 1) % 2].pose.position.x = enemy_buff_[(enemy_count_ + 2 - 1) % 2].pose.position.x-1.7*cosf(yaw);
        enemy_buff_[(enemy_count_ + 2 - 1) % 2].pose.position.y = enemy_buff_[(enemy_count_ + 2 - 1) % 2].pose.position.y-1.7*sinf(yaw);
        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
        blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
        SendGoal(enemy_buff_[(enemy_count_ + 2 - 1) % 2]);

      } else {
        blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
        blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);
        SendGoal(enemy_buff_[(enemy_count_ + 2 - 1) % 2]);
      }

//      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_RELATIVE_MODE);
//      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

    }
  }

  void SearchGoal () {

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

        search_x_min_ = (enemy_dis_1.pose.position.x > search_x_limit_) ? 4 : 0;
        search_x_max_ = (enemy_dis_1.pose.position.x > search_x_limit_) ? 8 : 4;
        search_x_max_ = (search_y_max_>costmap_2d_->GetSizeXCell()) ?  costmap_2d_->GetSizeXCell() : search_x_max_;

        search_y_min_ = (enemy_dis_1.pose.position.y > search_y_limit_) ? 2.5 : 0;
        search_y_max_ = (enemy_dis_1.pose.position.y > search_y_limit_) ? 5 : 2.5;
        search_y_max_ = (search_y_max_>costmap_2d_->GetSizeYCell()) ?  costmap_2d_->GetSizeYCell() : search_y_max_;

        yaw = std::atan2(y_diff, x_diff);

        auto orientation = tf::createQuaternionMsgFromYaw(yaw);

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position = enemy_dis_1.pose.position;
        goal.pose.orientation = orientation;
        SendGoal(goal);
        action_state_ = BehaviorState::RUNNING;
        lost_enemy_ = true;
        search_count_--;
      } else if (search_count_ != 0) {
        //RandomSearch();
        std::random_device rd;
        std::mt19937 gen(rd());

        std::uniform_real_distribution<float> x_uni_dis(search_x_min_, search_x_max_);
        std::uniform_real_distribution<float> y_uni_dis(search_y_min_, search_y_max_);

        float goal_x;
        float goal_y;
        unsigned int goal_cell_x;
        unsigned int goal_cell_y;
        while (true) {
          goal_x = x_uni_dis(gen);
          goal_y = y_uni_dis(gen);
          costmap_ptr_->GetCostMap()->World2Map(goal_x,
                                                goal_y,
                                                goal_cell_x,
                                                goal_cell_y);

          x_diff = goal_x - robot_map_pose_.pose.position.x;
          y_diff = goal_y - robot_map_pose_.pose.position.y;

          yaw = std::atan2(y_diff, x_diff);

          if (costmap_2d_->GetCost(goal_cell_x, goal_cell_y) >= 240 || std::abs(yaw) < 0.2) {
            continue;
          } else {
            break;
          }

        }

        auto orientation = tf::createQuaternionMsgFromYaw(yaw);

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = goal_x;
        goal.pose.position.y = goal_y;
        goal.pose.orientation = orientation;

        SendGoal(goal);
        action_state_ = BehaviorState::RUNNING;
        search_count_--;
      }
    }


  }

  void BuffGoal() {
    if (action_state_ != BehaviorState::RUNNING) {

      blackboard_ptr_->SetGimbalMode(GimbalMode::GIMBAL_PATROL_MODE);
      blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

      if (patrol_goals_.empty()) {
        LOG_ERROR << "patrol goal is empty";
        return;
      }
      SendGoal(buff_position_[buff_count_%buff_size_]);
      buff_count_ = ++buff_count_ % buff_size_;
      action_state_ = BehaviorState::RUNNING;
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
      action_state_ = BehaviorState::RUNNING;
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
      action_state_ = BehaviorState::RUNNING;
    }


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
      LOG_INFO<<"ACTIVE";
      action_state_ = BehaviorState::RUNNING;

    } else if (state == actionlib::SimpleClientGoalState::PENDING) {
      LOG_INFO<<"PENDING";
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
    global_planner_actionlib_client_.cancelGoal();
    local_planner_actionlib_client_.cancelGoal();
    action_state_ = BehaviorState::IDLE;
    blackboard_ptr_->SetChassisMode(ChassisMode::AUTO_SEPARATE_GIMBAL);

  }
  void CancelSearch() {
    search_count_ = 0;
  }

  void BackBootArea() {

    if (action_state_ != BehaviorState::RUNNING) {
//      auto dx = start_position_.pose.position.x - last_goal_.pose.position.x;
//      auto dy = start_position_.pose.position.y - last_goal_.pose.position.y;
//
//      auto start_yaw =  tf::getYaw(start_position_.pose.orientation);
//      auto last_yaw = tf::getYaw(last_goal_.pose.orientation);
//
//      auto d_yaw = start_yaw - last_yaw;

//      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.2) {
//        SendGoal(start_position_);
//        last_goal_ = start_position_;
//      }
      SendGoal(start_position_);
      action_state_ = BehaviorState::RUNNING;
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

    start_position_.header.stamp = ros::Time::now();
    start_position_.header.frame_id = "map";

    start_position_.pose.position.x = decision_config.start_position().x();
    start_position_.pose.position.y = decision_config.start_position().y();
    start_position_.pose.position.z = decision_config.start_position().z();

    auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.start_position().roll(),
                                                              decision_config.start_position().pitch(),
                                                              decision_config.start_position().yaw());

    start_position_.pose.orientation = quaternion;

  }

  void RandomSearch() {
    if (!lost_enemy_) {
      UpdateRobotPose();
      geometry_msgs::PoseStamped enemy_dis_1 = enemy_buff_[(enemy_count_ + 2 - 1) % 2];
//      geometry_msgs::PoseStamped enemy_dis_2 = enemy_buff_[(enemy_count_ + 2 - 2) % 2];
//      double time_diff = (enemy_dis_1.header.stamp - enemy_dis_2.header.stamp).toSec();
//      double x_diff = enemy_dis_1.pose.position.x - enemy_dis_2.pose.position.x;
//      double y_diff = enemy_dis_1.pose.position.y - enemy_dis_2.pose.position.y;

      double x_diff = enemy_dis_1.pose.position.x - robot_map_pose_.pose.position.x;
      double y_diff = enemy_dis_1.pose.position.y - robot_map_pose_.pose.position.y;

      search_x_min_ = (enemy_dis_1.pose.position.x > search_x_limit_) ? 4 : 0;
      search_x_max_ = (enemy_dis_1.pose.position.x > search_x_limit_) ? 8 : 4;
      search_x_max_ = (search_y_max_>costmap_2d_->GetSizeXCell()) ?  costmap_2d_->GetSizeXCell() : search_x_max_;

      search_y_min_ = (enemy_dis_1.pose.position.y > search_y_limit_) ? 2.5 : 0;
      search_y_max_ = (enemy_dis_1.pose.position.y > search_y_limit_) ? 5 : 2.5;
      search_y_max_ = (search_y_max_>costmap_2d_->GetSizeYCell()) ?  costmap_2d_->GetSizeYCell() : search_y_max_;

//      search_y_min_ = enemy_dis_1.pose.position.y;
//
//      search_y_max_ = enemy_dis_1.pose.position.y + y_diff * 5;

      yaw_ = std::atan2(y_diff, x_diff);
      lost_enemy_ = true;
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

  geometry_msgs::PoseStamped start_position_;
};

} // namespace rrts
} // namespace decision
#endif //MODULE_DECISION_VIC_GOAL_FACORY_H
