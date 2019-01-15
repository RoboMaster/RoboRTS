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

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_TEB_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_TEB_H

#include <mutex>

#include "io/io.h"
#include "state/error_code.h"
#include "alg_factory/algorithm_factory.h"

#include "costmap/costmap_interface.h"

#include "local_planner/local_planner_base.h"
#include "local_planner/optimal_base.h"
#include "local_planner/robot_position_cost.h"
#include "local_planner/local_visualization.h"
#include "local_planner/utility_tool.h"
#include "local_planner/odom_info.h"
#include "local_planner/data_converter.h"
#include "local_planner/data_base.h"
#include "local_planner/obstacle.h"
#include "local_planner/robot_footprint_model.h"

#include "timed_elastic_band/teb_vertex_pose.h"
#include "timed_elastic_band/teb_optimal.h"
#include "timed_elastic_band/proto/timed_elastic_band.pb.h"

namespace roborts_local_planner {

/**
 * @brief See local_planner_base.h
 */
class TebLocalPlanner : public LocalPlannerBase {
 public:
  TebLocalPlanner();
  ~TebLocalPlanner();
  roborts_common::ErrorInfo ComputeVelocityCommands(roborts_msgs::TwistAccel &cmd_vel) override;
  bool IsGoalReached () override;
  roborts_common::ErrorInfo Initialize (std::shared_ptr<roborts_costmap::CostmapInterface> local_cost,
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

  //! Tf listener
  std::weak_ptr<tf::TransformListener> tf_;
  //! Local cost map
  std::weak_ptr<roborts_costmap::CostmapInterface> local_cost_;

  //! Local planner frame(local planner will do optimal in this frame), different with global planner frame
  std::string global_frame_;
  //! Local planner costmap 2d
  roborts_costmap::Costmap2D *costmap_;
  //! Robot footprint cost
  std::shared_ptr<roborts_local_planner::RobotPositionCost> robot_cost_;
  //! Optimal based algorithm ptr
  OptimalBasePtr optimal_;
  //! Obstacle ptr
  std::vector<ObstaclePtr> obst_vector_;
  //! Must via point
  ViaPointContainer via_points_;
  //! Robot footprint
  std::vector<Eigen::Vector2d> robot_footprint_;
  //! Robot inscribed radius
  double robot_inscribed_radius_;
  //! Robot circumscribed radius
  double robot_circumscribed_radius;
  //! Robot odom info
  OdomInfo odom_info_;
  //! Global planner's solve
  nav_msgs::Path global_plan_, temp_plan_;
  //! Last velocity
  roborts_msgs::TwistAccel last_cmd_;
  //! Robot current velocity
  geometry_msgs::Twist robot_current_vel_;
  //! Robot current pose
  DataBase robot_pose_;
  //! Robot current pose
  tf::Stamped<tf::Pose> robot_tf_pose_;
  //! Robot goal
  DataBase robot_goal_;
  //! Visualize ptr use to visualize trajectory after optimize
  LocalVisualizationPtr visual_;
  //! Tf transform from global planner frame to optimal frame
  tf::StampedTransform plan_to_global_transform_;
  //! Way point after tf transform
  std::vector<DataBase> transformed_plan_;
  //! When no global planner give the global plan, use local goal express robot end point
  tf::Stamped<tf::Pose> local_goal_;
  //! Error info when running teb local planner algorithm
  roborts_common::ErrorInfo teb_error_info_;
  //! Call back function use to return error info
  ErrorInfoCallback error_callback_;
  //! Time begin when robot oscillation at a position
  std::chrono::system_clock::time_point oscillation_;
  //! Time allow robot oscillation at a position
  double oscillation_time_;
  //! Robot last position
  DataBase last_robot_pose_;
  //! Plan mutex
  std::mutex plan_mutex_;

  //! Optimal param
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
  //! Check if the algorithm is initialized
  bool is_initialized_ = false;
};

/**
 * Register the algorithm to algorithm factory
 */
roborts_common::REGISTER_ALGORITHM(LocalPlannerBase, "timed_elastic_band", TebLocalPlanner);

} // namespace roborts_local_planner




#endif // ROBORTS_PLANNING_LOCAL_PLANNER_TEB_H