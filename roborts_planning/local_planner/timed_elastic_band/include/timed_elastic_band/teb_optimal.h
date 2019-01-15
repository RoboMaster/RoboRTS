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

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_TEB_OPTIMAL_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_TEB_OPTIMAL_H
#include <cmath>
#include <chrono>

#include <limits>

#include <boost/thread.hpp>
#include <boost/thread/once.hpp>

#include <geometry_msgs/PoseStamped.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#include "local_planner/robot_position_cost.h"
#include "local_planner/optimal_base.h"
#include "local_planner/obstacle.h"
#include "local_planner/robot_footprint_model.h"
#include "local_planner/utility_tool.h"
#include "local_planner/local_visualization.h"
#include "local_planner/robot_position_cost.h"
#include "local_planner/data_base.h"

#include "timed_elastic_band/teb_vertex_console.h"
#include "timed_elastic_band/teb_acceleration_eage.h"
#include "timed_elastic_band/teb_kinematics_edge.h"
#include "timed_elastic_band/teb_obstacle_eage.h"
#include "timed_elastic_band/teb_prefer_rotdir_edge.h"
#include "timed_elastic_band/teb_time_optimal_eage.h"
#include "timed_elastic_band/teb_velocity_eage.h"
#include "timed_elastic_band/teb_via_point_edge.h"
#include "timed_elastic_band/proto/timed_elastic_band.pb.h"


namespace roborts_local_planner {

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  TebBlockSolver;
typedef g2o::LinearSolverCSparse<TebBlockSolver::PoseMatrixType> TebLinearSolver;
typedef std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > ViaPointContainer;
/**
 * @brief See optimal_base.h
 */
class TebOptimal : public OptimalBase {
 public:

  TebOptimal();

  TebOptimal (const Config& config_param, ObstContainer* obstacles = NULL,
              RobotFootprintModelPtr robot_model = boost::make_shared<PointRobotFootprint>(),
              LocalVisualizationPtr visual = LocalVisualizationPtr(),const ViaPointContainer* via_points = NULL);

  ~TebOptimal() {

  }

  void initialize(const Config& config_param, ObstContainer* obstacles = NULL,
                  RobotFootprintModelPtr robot_model = boost::make_shared<PointRobotFootprint>(),
                  LocalVisualizationPtr visual = LocalVisualizationPtr(),const ViaPointContainer* via_points = NULL);

  bool Optimal(std::vector<DataBase>& initial_plan, const geometry_msgs::Twist* start_vel = NULL,
               bool free_goal_vel = false, bool micro_control = false) override;

  bool Optimal(const DataBase& start, const DataBase& goal, const geometry_msgs::Twist* start_vel = NULL,
               bool free_goal_vel=false, bool micro_control = false) override;

  bool GetVelocity(roborts_common::ErrorInfo &error_info, double& vx, double& vy, double& omega,
                   double& acc_x, double& acc_y, double& acc_omega) const override ;

  bool OptimizeTeb(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards = false,
                   double obst_cost_scale=1.0, double viapoint_cost_scale=1.0, bool alternative_time_cost=false);

  void SetVisualization(LocalVisualizationPtr visualize);

  virtual void Visualize();

  void SetVelocityStart(const geometry_msgs::Twist& vel_start);

  void SetVelocityEnd(const geometry_msgs::Twist& vel_end);


  void SetVelocityGoalFree() {
    vel_end_.first = false;
  }

  void SetObstVector(ObstContainer* obst_vector) {
    obstacles_ = obst_vector;
  }

  const ObstContainer& GetObstVector() const {
    return *obstacles_;
  }

  void SetViaPoints(const ViaPointContainer* via_points) {
    via_points_ = via_points;
  }

  const ViaPointContainer& GetViaPoints() const {
    return *via_points_;
  }

  void ClearPlanner() override {
    ClearGraph();
  }

  virtual void SetPreferredTurningDir(RotType dir) {
    prefer_rotdir_=dir;
  }

  static void RegisterG2OTypes();

  boost::shared_ptr<g2o::SparseOptimizer> Optimizer() {
    return optimizer_;
  }

  bool IsOptimized() const {
    return optimized_;
  }

  void ComputeCurrentCost(double obst_cost_scale=1.0, double viapoint_cost_scale=1.0, bool alternative_time_cost=false);

  virtual void ComputeCurrentCost(std::vector<double>& cost, double obst_cost_scale=1.0,
                                  double viapoint_cost_scale=1.0, bool alternative_time_cost=false) {
    ComputeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
    cost.push_back( GetCurrentCost() );
  }

  double GetCurrentCost() const {
    return cost_;
  }

  inline void ExtractVelocity(const DataBase& pose1, const DataBase& pose2, double dt,
                              double& vx, double& vy, double& omega) const;

  void GetVelocityProfile(std::vector<geometry_msgs::Twist>& velocity_profile) const;

  //void GetFullTrajectory(std::vector<TrajectoryPointMsg>& trajectory) const;

  bool IsTrajectoryFeasible(roborts_common::ErrorInfo &error_info, RobotPositionCost* position_cost, const std::vector<Eigen::Vector2d>& footprint_spec,
                            double inscribed_radius = 0.0, double circumscribed_radius=0.0, int look_ahead_idx=-1) override ;

  bool IsHorizonReductionAppropriate(const std::vector<DataBase>& initial_plan) const override ;

 protected:
  bool BuildGraph(double weight_multiplier=1.0);

  bool OptimizeGraph(int no_iterations, bool clear_after=true);

  void ClearGraph();

  void AddTebVertices();

  void AddVelocityEdges();

  void AddAccelerationEdges();

  void AddTimeOptimalEdges();

  void AddObstacleEdges(double weight_multiplier=1.0);

  void AddObstacleLegacyEdges(double weight_multiplier=1.0);

  void AddViaPointsEdges();

  void AddPreferRotDirEdges();

  void AddKinematicsDiffDriveEdges();

  void AddKinematicsCarlikeEdges();

  void AddDynamicObstaclesEdges();



  boost::shared_ptr<g2o::SparseOptimizer> InitOptimizer();

  ObstContainer* obstacles_;
  const ViaPointContainer* via_points_;
  double cost_;
  RotType prefer_rotdir_;
  LocalVisualizationPtr visualization_;


  RobotFootprintModelPtr robot_model_;
  TebVertexConsole vertex_console_;
  boost::shared_ptr<g2o::SparseOptimizer> optimizer_;
  std::pair<bool, geometry_msgs::Twist> vel_start_;
  std::pair<bool, geometry_msgs::Twist> vel_end_;

  Robot        robot_info_;
  Config       param_config_;
  Obstacles    obstacles_info_;
  Trajectory   trajectory_info_;
  Optimization optimization_info_;

  bool initialized_;
  bool optimized_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


typedef boost::shared_ptr<TebOptimal> TebOptimalPlannerPtr;

} // namespace roborts_local_planner

#endif  // ROBORTS_PLANNING_LOCAL_PLANNER_TEB_OPTIMAL_H