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

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <modules/perception/map/costmap/costmap_interface.h>
#include "modules/planning/local_planner/timed_elastic_band/teb_local_planner.h"

namespace rrts {
namespace planning {
namespace local_planner {

TebLocalPlanner::TebLocalPlanner () {

}

TebLocalPlanner::~TebLocalPlanner () {

}

rrts::common::ErrorInfo TebLocalPlanner::ComputeVelocityCommands(geometry_msgs::Twist &cmd_vel) {

  if (!is_initialized_) {
    LOG_ERROR << "timed_elastic_band doesn't be initialized";
    rrts::common::ErrorInfo algorithm_init_error(rrts::common::LP_ALGORITHM_INITILIZATION_ERROR,
                                                 "teb initialize failed");
    LOG_ERROR << algorithm_init_error.error_msg();
    return algorithm_init_error;
  }


  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.angular.z = 0;

  UpdateRobotPose();
  UpdateRobotVel();
  UpdateGlobalToPlanTranform();

  auto time_now = std::chrono::system_clock::now();
  oscillation_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - oscillation_).count() / 1000.0f;

  if (oscillation_time_ > 1.0) {
    if ((robot_pose_.GetPosition() - last_robot_pose_.GetPosition()).norm() < 0.1) {
      local_cost_.lock()->ClearCostMap();
    } else {
      oscillation_time_ = 0;
      oscillation_ = std::chrono::system_clock::now();
      last_robot_pose_ = robot_pose_;
    }
  }

  if (IsGoalReached()) {
    rrts::common::ErrorInfo algorithm_ok(rrts::common::OK, "reached the goal");
    LOG_INFO << "reached the goal";
    return algorithm_ok;
  }

  PruneGlobalPlan();

  int goal_idx;

  if (!TransformGlobalPlan(&goal_idx)) {
    rrts::common::ErrorInfo PlanTransformError(rrts::common::LP_PLANTRANSFORM_ERROR, "plan transform error");
    LOG_ERROR << PlanTransformError.error_msg();
    return PlanTransformError;
  }

  if (transformed_plan_.poses.empty()) {
    rrts::common::ErrorInfo PlanTransformError(rrts::common::LP_PLANTRANSFORM_ERROR, "transformed plan is empty");
    LOG_ERROR << "transformed plan is empty";
    return PlanTransformError;
  }

  tf::Stamped<tf::Pose> goal_point;
  tf::poseStampedMsgToTF(transformed_plan_.poses.back(), goal_point);
  robot_goal_.GetPosition().coeffRef(0) = goal_point.getOrigin().getX();
  robot_goal_.GetPosition().coeffRef(1) = goal_point.getOrigin().getY();

  if (global_plan_.poses.size() - goal_idx < 5) {
    robot_goal_.GetTheta() = tf::getYaw(global_plan_.poses.back().pose.orientation);
    transformed_plan_.poses.back().pose.orientation = tf::createQuaternionMsgFromYaw(robot_goal_.GetTheta());
  } else if (global_plan_overwrite_orientation_) {
    robot_goal_.GetTheta() = EstimateLocalGoalOrientation(goal_point, goal_idx);
    transformed_plan_.poses.back().pose.orientation = tf::createQuaternionMsgFromYaw(robot_goal_.GetTheta());
  } else {
    robot_goal_.GetTheta() = tf::getYaw(goal_point.getRotation());
  }

  if (transformed_plan_.poses.size()==1) {// plan only contains the goal
    transformed_plan_.poses.insert(transformed_plan_.poses.begin(), geometry_msgs::PoseStamped()); // insert start (not yet initialized)
  }
  tf::poseTFToMsg(robot_tf_pose_, transformed_plan_.poses.front().pose); // update start;

  obst_vector_.clear();

  if (obst_vector_.empty()) {
    UpdateObstacleWithCostmap(robot_goal_.GetPosition());
  }

  UpdateViaPointsContainer();

  bool success = optimal_->Optimal(transformed_plan_, &robot_current_vel_, free_goal_vel_);

  if (!success) {
    optimal_->ClearPlanner();
    rrts::common::ErrorInfo OptimalError(rrts::common::LP_OPTIMAL_ERROR, "optimal error");
    LOG_ERROR << "optimal error";
    last_cmd_ = cmd_vel;
    return OptimalError;
  }

  bool feasible = optimal_->IsTrajectoryFeasible(teb_error_info_, robot_cost_.get(), robot_footprint_,
                                                 robot_inscribed_radius_, robot_circumscribed_radius,
                                                 fesiable_step_look_ahead_);
  if (!feasible) {
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    optimal_->ClearPlanner();
    last_cmd_ = cmd_vel;
    LOG_ERROR << "trajectory is not feasible";
    rrts::common::ErrorInfo trajectory_error(rrts::common::LP_ALGORITHM_TRAJECTORY_ERROR, "trajectory is not feasible");
    return trajectory_error;
  }

  if (!optimal_->GetVelocity(teb_error_info_, cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z)) {
    optimal_->ClearPlanner();
    LOG_ERROR << "can not get the velocity";
    rrts::common::ErrorInfo velocity_error(rrts::common::LP_VELOCITY_ERROR, "velocity is not feasible");
    last_cmd_ = cmd_vel;
    return velocity_error;
  }

  SaturateVelocity(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z,
                   max_vel_x_,max_vel_y_,max_vel_theta_,max_vel_x_backwards);

  last_cmd_ = cmd_vel;

  optimal_->Visualize();

  LOG_INFO << "compute velocity succeed";
  return rrts::common::ErrorInfo(rrts::common::ErrorCode::OK);
}

bool TebLocalPlanner::IsGoalReached () {

  tf::Stamped<tf::Pose> global_goal;
  tf::poseStampedMsgToTF(global_plan_.poses.back(), global_goal);
  global_goal.setData( plan_to_global_transform_ * global_goal );
  auto goal = DataConverter::LocalConvertTFData(global_goal);

  auto distance = (goal.first - robot_pose_.GetPosition()).norm();
  double delta_orient = g2o::normalize_theta( goal.second - robot_pose_.GetTheta());

  if (distance < xy_goal_tolerance_
      && fabs(delta_orient) < yaw_goal_tolerance_) {
    LOG_INFO << "goal reached";
    return true;
  } else {
    return false;
  }

}

bool TebLocalPlanner::SetPlan(const nav_msgs::Path& plan, const geometry_msgs::PoseStamped& goal) {

  LOG_INFO << "set plan";
  if (plan.poses.empty()) {
    global_plan_.poses.push_back(goal);
  } else {
    global_plan_ = plan;
  }
}

bool TebLocalPlanner::PruneGlobalPlan() {
  if (global_plan_.poses.empty()) {
    return true;
  }
  try {
    tf::StampedTransform global_to_plan_transform;
    tf_.lock()->lookupTransform(global_plan_.poses.front().header.frame_id,
                                robot_tf_pose_.frame_id_, ros::Time(0),
                                global_to_plan_transform);
    tf::Stamped<tf::Pose> robot;
    robot.setData( global_to_plan_transform * robot_tf_pose_ );

    Eigen::Vector2d robot_to_goal(robot.getOrigin().x() - global_plan_.poses.back().pose.position.x,
                                  robot.getOrigin().y() - global_plan_.poses.back().pose.position.y);


    for (auto iterator = global_plan_.poses.begin(); iterator != global_plan_.poses.end(); ++iterator) {
      Eigen::Vector2d temp_vector (robot.getOrigin().x() - iterator->pose.position.x,
                                   robot.getOrigin().y() - iterator->pose.position.y);
      if (temp_vector.norm() < 0.8) {
        if (iterator == global_plan_.poses.begin()) {
          break;
        }
        global_plan_.poses.erase(global_plan_.poses.begin(), iterator);
        break;
      }
    }
  }
  catch (const tf::TransformException& ex) {
   LOG_ERROR << "prune global plan false, " << ex.what();
    return false;
  }
  return true;
}

bool TebLocalPlanner::TransformGlobalPlan(int *current_goal_idx) {


  transformed_plan_.poses.clear();

  try {
    if (global_plan_.poses.empty()) {
      LOG_ERROR << "Received plan with zero length";
      *current_goal_idx = 0;
      return false;
    }
    UpdateGlobalToPlanTranform();

    tf::Stamped<tf::Pose> robot_pose;
    tf_.lock()->transformPose(global_plan_.poses.front().header.frame_id, robot_tf_pose_, robot_pose);

    double dist_threshold = std::max(costmap_->GetSizeXCell() * costmap_->GetResolution() / 2.0,
                                     costmap_->GetSizeYCell() * costmap_->GetResolution() / 2.0);
    dist_threshold *= 0.85;

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;
    double new_sq_dist = 0;
    while (i < (int)global_plan_.poses.size()) {
      double x_diff = robot_pose.getOrigin().x() - global_plan_.poses[i].pose.position.x;
      double y_diff = robot_pose.getOrigin().y() - global_plan_.poses[i].pose.position.y;
      new_sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold) {
        sq_dist = new_sq_dist;
        break;
      }
      sq_dist = new_sq_dist;
      ++i;
    }

    tf::Stamped<tf::Pose> tf_pose;
    geometry_msgs::PoseStamped newer_pose;

    double plan_length = 0;

    while(i < (int)global_plan_.poses.size() &&
        sq_dist <= sq_dist_threshold && (cut_lookahead_dist_<=0 ||
        plan_length <= cut_lookahead_dist_)) {
      const geometry_msgs::PoseStamped& pose = global_plan_.poses[i];
      tf::poseStampedMsgToTF(pose, tf_pose);
      tf_pose.setData(plan_to_global_transform_ * tf_pose);
      tf_pose.stamp_ = plan_to_global_transform_.stamp_;
      tf_pose.frame_id_ = global_frame_;
      tf::poseStampedTFToMsg(tf_pose, newer_pose);

      transformed_plan_.poses.push_back(newer_pose);

      double x_diff = robot_pose.getOrigin().x() - global_plan_.poses[i].pose.position.x;
      double y_diff = robot_pose.getOrigin().y() - global_plan_.poses[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;

      if (i>0 && cut_lookahead_dist_>0) {
        plan_length += Distance(global_plan_.poses[i-1].pose.position.x, global_plan_.poses[i-1].pose.position.y,
                                global_plan_.poses[i].pose.position.x, global_plan_.poses[i].pose.position.y);
      }

      ++i;
    }

    if (transformed_plan_.poses.empty()) {
      tf::poseStampedMsgToTF(global_plan_.poses.back(), tf_pose);
      tf_pose.setData(plan_to_global_transform_ * tf_pose);
      tf_pose.stamp_ = plan_to_global_transform_.stamp_;
      tf_pose.frame_id_ = global_frame_;
      tf::poseStampedTFToMsg(tf_pose, newer_pose);

      transformed_plan_.poses.push_back(newer_pose);

      if (current_goal_idx) {
        *current_goal_idx = int(global_plan_.poses.size())-1;
      }
    } else {
      if (current_goal_idx) {
        *current_goal_idx = i-1;
      }
    }

  }
  catch(tf::LookupException& ex) {
    LOG_ERROR << "transform error, " << ex.what();
    return false;
  }
  catch(tf::ConnectivityException& ex) {
    LOG_ERROR << "Connectivity Error: " <<  ex.what();
    return false;
  }
  catch(tf::ExtrapolationException& ex) {
    LOG_ERROR << "Extrapolation Error: " <<  ex.what();
    if (global_plan_.poses.size() > 0) {
      LOG_INFO << "Global Frame: " << global_frame_.c_str() << "Plan Frame size "
                << (unsigned int)global_plan_.poses.size() << ": " << global_plan_.poses[0].header.frame_id.c_str();
    }
    return false;
  }

  return true;
}

double TebLocalPlanner::EstimateLocalGoalOrientation(const tf::Stamped<tf::Pose>& local_goal,
                                                     int current_goal_idx, int moving_average_length) const {
  int n = (int)global_plan_.poses.size();


  if (current_goal_idx > n-moving_average_length-2) {
    if (current_goal_idx >= n-1) {
      return tf::getYaw(local_goal_.getRotation());
    } else {
      tf::Quaternion global_orientation;
      tf::quaternionMsgToTF(global_plan_.poses.back().pose.orientation, global_orientation);
      return  tf::getYaw(plan_to_global_transform_.getRotation() *  global_orientation );
    }
  }

  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 );

  std::vector<double> candidates;
  tf::Stamped<tf::Pose> tf_pose_k = local_goal;
  tf::Stamped<tf::Pose> tf_pose_kp1;

  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i) {

    const geometry_msgs::PoseStamped& pose = global_plan_.poses.at(i+1);
    tf::poseStampedMsgToTF(pose, tf_pose_kp1);
    tf_pose_kp1.setData(plan_to_global_transform_ * tf_pose_kp1);

    candidates.push_back( std::atan2(tf_pose_kp1.getOrigin().getY() - tf_pose_k.getOrigin().getY(),
                                     tf_pose_kp1.getOrigin().getX() - tf_pose_k.getOrigin().getX() ) );

    if (i<range_end-1)
      tf_pose_k = tf_pose_kp1;
  }
  return AverageAngles(candidates);
}

void TebLocalPlanner::UpdateObstacleWithCostmap(Eigen::Vector2d local_goal) {

  //Eigen::Vector2d robot_orient = robot_pose_.OrientationUnitVec();
  Eigen::Vector2d goal_orient = local_goal - robot_pose_.GetPosition();

  for (unsigned int i=0; i<costmap_->GetSizeXCell()-1; ++i) {
    for (unsigned int j=0; j<costmap_->GetSizeYCell()-1; ++j) {
      if (costmap_->GetCost(i,j) == rrts::planning::local_planner::LETHAL_OBSTACLE) {
        Eigen::Vector2d obs;
        costmap_->Map2World(i, j, obs.coeffRef(0), obs.coeffRef(1));

        Eigen::Vector2d obs_dir = obs - robot_pose_.GetPosition();
        if (obs_dir.dot(goal_orient) < 0
            && obs_dir.norm() > osbtacle_behind_robot_dist_) {
          continue;
        }

        obst_vector_.push_back(ObstaclePtr(new PointObstacle(obs)));
      }
    } // for y cell size
  }  // for x cell size

}

void TebLocalPlanner::UpdateViaPointsContainer() {

  via_points_.clear();

  double min_separation = param_config_.trajectory_opt().global_plan_viapoint_sep();
  if (min_separation<0) {
    return;
  }

  std::size_t prev_idx = 0;
  for (std::size_t i=1; i < transformed_plan_.poses.size(); ++i) {// skip first one, since we do not need any point before the first min_separation [m]

    if (Distance( transformed_plan_.poses[prev_idx].pose.position.x, transformed_plan_.poses[prev_idx].pose.position.y,
                  transformed_plan_.poses[i].pose.position.x, transformed_plan_.poses[i].pose.position.y ) < min_separation) {
      continue;
    }

    via_points_.push_back( Eigen::Vector2d( transformed_plan_.poses[i].pose.position.x, transformed_plan_.poses[i].pose.position.y ) );
    prev_idx = i;
  }
}

void TebLocalPlanner::UpdateRobotPose() {
  local_cost_.lock()->GetRobotPose(robot_tf_pose_);
  Eigen::Vector2d position;
  position.coeffRef(0) = robot_tf_pose_.getOrigin().getX();
  position.coeffRef(1) = robot_tf_pose_.getOrigin().getY();
  robot_pose_ = DataBase(position, tf::getYaw(robot_tf_pose_.getRotation()));
}

void TebLocalPlanner::UpdateRobotVel() {
  tf::Stamped<tf::Pose> robot_vel_tf;
  odom_info_.GetVel(robot_vel_tf);
  robot_current_vel_.linear.x = robot_vel_tf.getOrigin().getX();
  robot_current_vel_.linear.y = robot_vel_tf.getOrigin().getY();
  robot_current_vel_.angular.z = tf::getYaw(robot_vel_tf.getRotation());
}

void TebLocalPlanner::UpdateGlobalToPlanTranform() {
  tf_.lock()->waitForTransform(global_frame_, ros::Time::now(),
                               global_plan_.poses.front().header.frame_id, global_plan_.poses.front().header.stamp,
                               global_plan_.poses.front().header.frame_id, ros::Duration(0.5));
  tf_.lock()->lookupTransform(global_frame_, ros::Time(),
                              global_plan_.poses.front().header.frame_id, global_plan_.poses.front().header.stamp,
                              global_plan_.poses.front().header.frame_id, plan_to_global_transform_);
}

void TebLocalPlanner::SaturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y,
                      double max_vel_theta, double max_vel_x_backwards) const {

  if (vx > max_vel_x) {
    vx = max_vel_x;
  }

  if (max_vel_x_backwards<=0) {
    LOG_INFO << "Do not choose max_vel_x_backwards to be <=0. "
        "Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.";
  } else if (vx < -max_vel_x_backwards) {
    vx = -max_vel_x_backwards;
  }

  if (vy > max_vel_y) {
    vy = max_vel_y;
  } else if (vy < -max_vel_y) {
    vy = -max_vel_y;
  }

  if (omega > max_vel_theta) {
    omega = max_vel_theta;
  } else if (omega < -max_vel_theta) {
    omega = -max_vel_theta;
  }

}

double TebLocalPlanner::ConvertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const {
  if (omega==0 || v==0) {
    return 0;
  }

  double radius = v/omega;

  if (fabs(radius) < min_turning_radius) {
    radius = double(g2o::sign(radius)) * min_turning_radius;
  }

  return std::atan(wheelbase / radius);
}

rrts::common::ErrorInfo TebLocalPlanner::Initialize (std::shared_ptr<rrts::perception::map::CostmapInterface> local_cost,
                                  std::shared_ptr<tf::TransformListener> tf, LocalVisualizationPtr visual) {

  if (!is_initialized_) {
    oscillation_ = std::chrono::system_clock::now();
    tf_ = tf;
    local_cost_ = local_cost;
    rrts::common::ReadProtoFromTextFile("modules/planning/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt", &param_config_);
    if (&param_config_ == nullptr) {
      LOG_ERROR << "error occur when loading config file";
      rrts::common::ErrorInfo read_file_error(rrts::common::ErrorCode::LP_ALGORITHM_INITILIZATION_ERROR,
                                              "load algorithm param file failed");
      is_initialized_ = false;
      LOG_ERROR << read_file_error.error_msg();
      return read_file_error;
    }

    max_vel_x_          = param_config_.kinematics_opt().max_vel_x();
    max_vel_y_          = param_config_.kinematics_opt().max_vel_y();
    max_vel_theta_      = param_config_.kinematics_opt().max_vel_theta();
    max_vel_x_backwards = param_config_.kinematics_opt().max_vel_x_backwards();
    free_goal_vel_      = param_config_.tolerance_opt().free_goal_vel();

    xy_goal_tolerance_  = param_config_.tolerance_opt().xy_goal_tolerance();
    yaw_goal_tolerance_ = param_config_.tolerance_opt().yaw_goal_tolerance();

    cut_lookahead_dist_       = param_config_.trajectory_opt().max_global_plan_lookahead_dist();
    fesiable_step_look_ahead_ = param_config_.trajectory_opt().feasibility_check_no_poses();

    osbtacle_behind_robot_dist_ = param_config_.obstacles_opt().costmap_obstacles_behind_robot_dist();

    global_plan_overwrite_orientation_ = param_config_.trajectory_opt().global_plan_overwrite_orientation();

    global_frame_ = local_cost_.lock()->GetGlobalFrameID();

    visual_ = visual;

    costmap_ = local_cost_.lock()->GetLayeredCostmap()->GetCostMap();

    robot_cost_ = std::make_shared<rrts::planning::local_planner::RobotPositionCost>(*costmap_);

    obst_vector_.reserve(200);
    RobotFootprintModelPtr robot_model = GetRobotFootprintModel(param_config_);
    optimal_ = OptimalBasePtr(new TebOptimal(param_config_, &obst_vector_, robot_model, visual_, &via_points_));

    std::vector<geometry_msgs::Point> robot_footprint;
    robot_footprint = local_cost_.lock()->GetRobotFootprint();
    //robot_footprint_ = DataConverter::LocalConvertGData(robot_footprint);

    rrts::perception::map::RobotPose robot_pose;
    local_cost_.lock()->GetRobotPose(robot_pose);
    auto temp_pose = DataConverter::LocalConvertRMData(robot_pose);
    robot_pose_ = DataBase(temp_pose.first, temp_pose.second);
    robot_footprint_.push_back(robot_pose_.GetPosition());
    last_robot_pose_ = robot_pose_;

    RobotPositionCost::CalculateMinAndMaxDistances(robot_footprint_,
                                                   robot_inscribed_radius_,
                                                   robot_circumscribed_radius);
    odom_info_.SetTopic(param_config_.opt_frame().odom_frame());

    is_initialized_ = true;
    LOG_INFO << "local algorithm initialize ok";
    return rrts::common::ErrorInfo(rrts::common::ErrorCode::OK);
  }

  return rrts::common::ErrorInfo(rrts::common::ErrorCode::OK);
}

void TebLocalPlanner::RegisterErrorCallBack(ErrorInfoCallback error_callback) {
  error_callback_ = error_callback;
}

bool TebLocalPlanner::CutAndTransformGlobalPlan(int *current_goal_idx) {
  if (!transformed_plan_.poses.empty()) {
    transformed_plan_.poses.clear();
  }

}

bool TebLocalPlanner::SetPlanOrientation() {
  if (global_plan_.poses.size() < 2) {
    LOG_WARNING << "can not compute the orientation because the global plan size is: " << global_plan_.poses.size();
    return false;
  } else {
    //auto goal = DataConverter::LocalConvertGData(global_plan_.poses.back().pose);
    //auto line_vector = (robot_pose_.GetPosition() - goal.first);
    //auto  orientation = GetOrientation(line_vector);
    for (int i = 0; i < global_plan_.poses.size() - 1; ++i) {
      auto pose = DataConverter::LocalConvertGData(global_plan_.poses[i].pose);
      auto next_pose = DataConverter::LocalConvertGData(global_plan_.poses[i+1].pose);
      double x = global_plan_.poses[i+1].pose.position.x - global_plan_.poses[i].pose.position.x;
      double y = global_plan_.poses[i+1].pose.position.y - global_plan_.poses[i].pose.position.y;
      double angle = atan2(y, x);
      auto quaternion = EulerToQuaternion(0, 0, angle);
      global_plan_.poses[i].pose.orientation.w = quaternion[0];
      global_plan_.poses[i].pose.orientation.x = quaternion[1];
      global_plan_.poses[i].pose.orientation.y = quaternion[2];
      global_plan_.poses[i].pose.orientation.z = quaternion[3];
    }
  }
}

} // namespace local_planner
} // namespace planning
} // namespace rrts