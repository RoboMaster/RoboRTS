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
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
* Author: Eitan Marder-Eppstein
*********************************************************************/

#ifndef ROBORTS_PLANNING_BASE_LOCAL_PLANNER_H
#define ROBORTS_PLANNING_BASE_LOCAL_PLANNER_H

#include <functional>

#include <boost/shared_ptr.hpp>

#include <tf/transform_listener.h>

#include "state/error_code.h"
#include "costmap/costmap_interface.h"
#include "local_planner/local_visualization.h"
#include "roborts_msgs/TwistAccel.h"

/**
 * @brief parent class of all local planner algorithms
 */
namespace roborts_local_planner {

//! Error info callback function
typedef boost::function<void (const roborts_common::ErrorInfo &)> ErrorInfoCallback;
class LocalPlannerBase {
 public:
  /**
   * @brief Virtual function calculate robot velocity should be implemented by derived class
   * @param cmd_vel Velocity use to control robot
   * @return Error info
   */
  virtual roborts_common::ErrorInfo ComputeVelocityCommands(roborts_msgs::TwistAccel &cmd_vel) = 0;

  /**
   * @brief Virtual function judge if robot reach the goal should be implemented by derived class
   * @return If true reached the goal, else not
   */
  virtual bool IsGoalReached() = 0;

  /**
   * @brief Virtual function initialize local planner algorithm should be implemented by derived class
   * @param local_cost Local cost map
   * @param tf Tf listener
   * @param visual Visualize pointer
   * @return Error info
   */
  virtual roborts_common::ErrorInfo Initialize(std::shared_ptr<roborts_costmap::CostmapInterface> local_cost,
                                             std::shared_ptr<tf::TransformListener> tf, LocalVisualizationPtr visual) = 0;

  /**
   * @brief Virtual function Set global plan's result to local planner or set a goal to local planner
   * should be implement by derived class
   * @param plan Result of global planner
   * @param goal Goal of local planner
   * @return If true success, else fail
   */
  virtual bool SetPlan(const nav_msgs::Path& plan, const geometry_msgs::PoseStamped& goal) = 0;

  /**
   * @brief Virtual function Register error callback function should be implemented by derived class
   * @param error_callback Callback function
   */
  virtual void RegisterErrorCallBack(ErrorInfoCallback error_callback) = 0;
  virtual ~LocalPlannerBase(){}

 protected:
  LocalPlannerBase () {}
};

typedef boost::shared_ptr<LocalPlannerBase> LocalPlannerPtr;

} // namespace roborts_local_planner




#endif //ROBORTS_PLANNING_BASE_LOCAL_PLANNER_H
