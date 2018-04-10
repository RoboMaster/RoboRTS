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

#ifndef MODULES_PLANNING_LOCAL_PLANNER_TEB_H
#define MODULES_PLANNING_LOCAL_PLANNER_TEB_H

#include <mutex>

#include "common/io.h"
#include "common/algorithm_factory.h"

#include "modules/perception/map/costmap/costmap_interface.h"

#include "modules/planning/local_planner/local_planner_base.h"
#include "modules/planning/local_planner/optimal_base.h"
#include "modules/planning/local_planner/robot_position_cost.h"
#include "modules/planning/local_planner/local_visualization.h"
#include "modules/planning/local_planner/utility_tool.h"
#include "modules/planning/local_planner/odom_info.h"

#include "modules/planning/local_planner/timed_elastic_band/teb_vertex_pose.h"
#include "modules/planning/local_planner/timed_elastic_band/teb_optimal.h"
#include "modules/planning/local_planner/timed_elastic_band/proto/timed_elastic_band.pb.h"

namespace rrts {
namespace planning {
namespace local_planner {

class TebLocalPlanner : public LocalPlannerBase {
 public:
  TebLocalPlanner();
  ~TebLocalPlanner();
  rrts::common::ErrorInfo ComputeVelocityCommands(geometry_msgs::Twist &cmd_vel) override;
  bool IsGoalReached () override;
  rrts::common::ErrorInfo Initialize (std::shared_ptr<rrts::perception::map::CostmapInterface> local_cost,
                   std::shared_ptr<tf::TransformListener> tf, LocalVisualizationPtr visual) override;
  bool SetPlan(const nav_msgs::Path& plan, const geometry_msgs::PoseStamped& goal) override ;
  bool GetPlan(const nav_msgs::Path& plan);
  bool SetPlanOrientation();
  void RegisterErrorCallBack(ErrorInfoCallback error_callback) override;

  bool PruneGlobalPlan();

  bool TransformGlobalPlan(int *current_goal_idx = NULL);

  double EstimateLocalGoalOrientation(const DataBase& local_goal,
                                      int current_goal_idx, int moving_average_length=3) const;

  void UpdateViaPointsContainer();

  void SaturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y,
                        double max_vel_theta, double max_vel_x_backwards) const;

  void UpdateObstacleWithCostmap(Eigen::Vector2d local_goal);
  void UpdateRobotPose();
  void UpdateRobotVel();
  void UpdateGlobalToPlanTranform();
  bool CutAndTransformGlobalPlan(int *current_goal_idx = NULL);

  double ConvertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius = 0) const;

  std::weak_ptr<tf::TransformListener> tf_;
  std::weak_ptr<rrts::perception::map::CostmapInterface> local_cost_;

  std::string global_frame_;
  rrts::perception::map::Costmap2D *costmap_;
  std::shared_ptr<rrts::planning::local_planner::RobotPositionCost> robot_cost_;
  OptimalBasePtr optimal_;
  std::vector<ObstaclePtr> obst_vector_;
  ViaPointContainer via_points_;
  std::vector<Eigen::Vector2d> robot_footprint_;
  double robot_inscribed_radius_;
  double robot_circumscribed_radius;
  OdomInfo odom_info_;
  nav_msgs::Path global_plan_, temp_plan_;
  geometry_msgs::Twist last_cmd_, robot_current_vel_;
  DataBase robot_pose_;
  tf::Stamped<tf::Pose> robot_tf_pose_;
  DataBase robot_goal_;
  LocalVisualizationPtr visual_;
  tf::StampedTransform plan_to_global_transform_;
  std::vector<DataBase> transformed_plan_;
  tf::Stamped<tf::Pose> local_goal_;
  rrts::common::ErrorInfo teb_error_info_;
  ErrorInfoCallback error_callback_;
  std::chrono::system_clock::time_point oscillation_;
  double oscillation_time_;
  DataBase last_robot_pose_;
  std::mutex plan_mutex_;


  int ijk = 0;

  Config param_config_;
  bool  free_goal_vel_;
  bool  global_plan_overwrite_orientation_;
  float cut_lookahead_dist_;
  long  fesiable_step_look_ahead_;
  float max_vel_x_;
  float max_vel_y_;
  float max_vel_theta_;
  float max_vel_x_backwards;
  float xy_goal_tolerance_;
  float yaw_goal_tolerance_;
  float osbtacle_behind_robot_dist_;


 protected:
  bool is_initialized_ = false;
};

rrts::common::REGISTER_ALGORITHM(LocalPlannerBase, "timed_elastic_band", TebLocalPlanner);

} // namespace local_planner
} // namespace planning
} // namespace rrts




#endif // MODULES_PLANNING_LOCAL_PLANNER_TEB_H