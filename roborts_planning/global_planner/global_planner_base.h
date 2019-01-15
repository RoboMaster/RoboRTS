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
#ifndef ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_BASE_H
#define ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_BASE_H

#include "state/error_code.h"

#include "costmap/costmap_interface.h"

namespace roborts_global_planner{

class GlobalPlannerBase {
 public:
  typedef std::shared_ptr<roborts_costmap::CostmapInterface> CostmapPtr;

  GlobalPlannerBase(CostmapPtr costmap_ptr)
      : costmap_ptr_(costmap_ptr) {
  };
  virtual ~GlobalPlannerBase() = default;

  virtual roborts_common::ErrorInfo Plan(const geometry_msgs::PoseStamped &start,
                                       const geometry_msgs::PoseStamped &goal,
                                       std::vector<geometry_msgs::PoseStamped> &path) = 0;

 protected:
  CostmapPtr costmap_ptr_;
};

} //namespace roborts_global_planner


#endif // ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_BASE_H
