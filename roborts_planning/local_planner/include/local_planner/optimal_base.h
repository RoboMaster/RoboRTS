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

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_OPTIMAL_BASE_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_OPTIMAL_BASE_H

#include <boost/shared_ptr.hpp>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "local_planner/data_base.h"
#include "state/error_code.h"
#include "local_planner/robot_position_cost.h"

namespace roborts_local_planner {

/**
 * @brief Base class of optimal based planner algorithm
 */
class OptimalBase {
 public:
  /**
   * @brief Constructor
   */
  OptimalBase(){

  }
  /**
   * @brief Entry function of optimal based planner algorithm
   * @param initial_plan Input and output, way point want to optimal
   * @param start_vel Start velocity of robot
   * @param free_goal_vel If true set the goal velocity to zero
   * @param micro_control If true when the goal behind the robot, robot will have negative x velocity
   * @return True if success, otherwise false
   */
  virtual bool Optimal(std::vector<DataBase>& initial_plan, const geometry_msgs::Twist* start_vel = NULL,
                    bool free_goal_vel = false, bool micro_control = false) = 0;

  /**
   * @brief Entry function of optimal based planner algorithm
   * @param start Start position (maybe robot current position)
   * @param goal Goal position (where robot want to go)
   * @param start_vel Robot current velocity
   * @param free_goal_vel If true set the goal velocity to zero
   * @param micro_control If true when the goal behind the robot, robot will have negative x velocity
   * @return True if success, otherwise false
   */
  virtual bool Optimal(const DataBase& start, const DataBase& goal, const geometry_msgs::Twist* start_vel = NULL,
                    bool free_goal_vel = false, bool micro_control = false) = 0;

  /**
   * @brief Get next control velocity
   * @param error_info Error info when get the robot velocity
   * @param vx X component velocity
   * @param vy Y component velocity
   * @param omega Angular velocity
   * @param acc_x X component acceleration
   * @param acc_y Y component acceleration
   * @param acc_omega Angular acceleration
   * @return True if success, otherwise false
   */
  virtual bool GetVelocity(roborts_common::ErrorInfo &error_info, double& vx, double& vy, double& omega,
                           double& acc_x, double& acc_y, double& acc_omega) const = 0;

  /**
   * @brief Clear all vertices and edges
   */
  virtual void ClearPlanner() = 0;

  /**
   * @brief Visualize trajectory and pose after optimize
   */
  virtual void Visualize() {

  }

  /**
   * @brief Judge if the trajectory after optimize is feasible
   * @param error_info Error info about trajectory feasible judge
   * @param position_cost Cost map which to calculate the point cost
   * @param footprint_spec Robot vertices
   * @param inscribed_radius Robot inscribed radius
   * @param circumscribed_radius Robot circumscribed radius
   * @param look_ahead_idx How many point want to check
   * @return True if feasible, otherwise is false
   */
  virtual bool IsTrajectoryFeasible(roborts_common::ErrorInfo &error_info, RobotPositionCost* position_cost,
                                    const std::vector<Eigen::Vector2d>& footprint_spec,
                                    double inscribed_radius = 0.0, double circumscribed_radius=0.0,
                                    int look_ahead_idx = -1) = 0;

  /**
   *@brief Now we not use
   */
  virtual bool IsHorizonReductionAppropriate(const std::vector<DataBase>& initial_plan) const {
    return false;
  }

  /**
   * @brief Compute current graph cost(errors)
   * @param cost Input and output, total errors of all edges
   * @param obst_cost_scale Obstacle's scale number
   * @param alternative_time_cost True use trajectory time diff add to errors
   */
  virtual void ComputeCurrentCost(std::vector<double>& cost, double obst_cost_scale = 1.0, bool alternative_time_cost = false) {

  }

  /**
   * @brief Destructor
   */
  virtual ~OptimalBase(){

  }


};

typedef boost::shared_ptr<OptimalBase> OptimalBasePtr;

} // namespace roborts_local_planner

#endif // ROBORTS_PLANNING_LOCAL_PLANNER_OPTIMAL_BASE_H