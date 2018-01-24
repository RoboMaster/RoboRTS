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

#ifndef MODULES_PLANNING_BASE_LOCAL_PLANNER_H
#define MODULES_PLANNING_BASE_LOCAL_PLANNER_H

#include <functional>

#include <boost/shared_ptr.hpp>

#include <tf/transform_listener.h>

#include "common/error_code.h"
#include "modules/perception/map/costmap/costmap_interface.h"
#include "modules/planning/local_planner/local_visualization.h"


/**
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
*/

namespace rrts {
namespace planning {
namespace local_planner {

typedef boost::function<void (const rrts::common::ErrorInfo &)> ErrorInfoCallback;
class LocalPlannerBase {
 public:
  virtual rrts::common::ErrorInfo ComputeVelocityCommands(geometry_msgs::Twist &cmd_vel) = 0;
  virtual bool IsGoalReached() = 0;
  virtual rrts::common::ErrorInfo Initialize(std::shared_ptr<rrts::perception::map::CostmapInterface> local_cost,
                                             std::shared_ptr<tf::TransformListener> tf, LocalVisualizationPtr visual) = 0;
  virtual bool SetPlan(const nav_msgs::Path& plan, const geometry_msgs::PoseStamped& goal) = 0;
  virtual void RegisterErrorCallBack(ErrorInfoCallback error_callback) = 0;
  virtual ~LocalPlannerBase(){}

  bool is_get_new_goal = false;

 protected:
  LocalPlannerBase () {}
};

typedef boost::shared_ptr<LocalPlannerBase> LocalPlannerPtr;

} // namespace local_planner
} // namespace planning
} // namespace rrts




#endif //MODULES_PLANNING_BASE_LOCAL_PLANNER_H
