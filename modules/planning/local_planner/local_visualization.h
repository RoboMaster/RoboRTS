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
#ifndef MODULES_PLANNING_LOCAL_PLANNER_LOCAL_VISUALIZATION_H
#define MODULES_PLANNING_LOCAL_PLANNER_LOCAL_VISUALIZATION_H

#include <iterator>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <ros/publisher.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

#include "modules/planning/local_planner/timed_elastic_band/teb_vertex_console.h"


namespace rrts {
namespace planning {
namespace local_planner {


class LocalVisualization {
 public:
  LocalVisualization();
  LocalVisualization(ros::NodeHandle& nh, const std::string& visualize_frame);
  void Initialization(ros::NodeHandle& nh, const std::string& visualize_frame);
  void PublishLocalPlan(const TebVertexConsole &vertex_console) const;

 protected:

  ros::Publisher local_planner_;
  ros::Publisher pose_pub_;

  std::string visual_frame_ = "map";

  bool initialized_;

};

typedef boost::shared_ptr<const LocalVisualization> LocalVisualizationPtr;

} // namespace local_planner
} // namespace planning
} // namespace rrts

#endif // MODULES_PLANNING_LOCAL_PLANNER_LOCAL_VISUALIZATION_H

