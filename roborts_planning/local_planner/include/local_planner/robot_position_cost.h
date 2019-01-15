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

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_ROBOT_POSITION_COST_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_ROBOT_POSITION_COST_H

#include <vector>

#include <Eigen/Core>
#include <boost/thread.hpp>

#include "costmap/costmap_interface.h"
#include "local_planner/utility_tool.h"
#include "local_planner/line_iterator.h"
#include "local_planner/robot_footprint_model.h"

namespace roborts_local_planner {

static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char FREE_SPACE = 0;

/**
 * @brief Calculate each robot vertices cost
 */
class RobotPositionCost {
 public:
  /**
   * @brief Constructor
   * @param cost_map Cost map use to calculate cost
   */
  RobotPositionCost(const roborts_costmap::Costmap2D& cost_map);
  /**
   * @brief Destructor
   */
  ~RobotPositionCost();

  /**
   * @brief Calculate minimum and maximum distance
   * @param footprint Robot vertices
   * @param min_dist Input and output, minimum distance
   * @param max_dist Input and output, maximum distance
   */
  static void CalculateMinAndMaxDistances (const std::vector<Eigen::Vector2d>& footprint, double& min_dist, double& max_dist) {
    min_dist = std::numeric_limits<double>::max();
    max_dist = 0.0;

    if (footprint.size() <= 2)
    {
      return;
    }

    for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
      double vertex_dist = Distance (0.0, 0.0, footprint[i].coeffRef(0), footprint[i].coeffRef(1));
      double edge_dist = PointToLineDistance(0.0, 0.0, footprint[i].coeffRef(0), footprint[i].coeffRef(1),
                                                footprint[i + 1].coeffRef(0), footprint[i + 1].coeffRef(1));
      min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
      max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
    }

    double vertex_dist = Distance(0.0, 0.0, footprint.back().coeffRef(0), footprint.back().coeffRef(1));
    double edge_dist = PointToLineDistance(0.0, 0.0, footprint.back().coeffRef(0), footprint.back().coeffRef(1),
                                              footprint.front().coeffRef(0), footprint.front().coeffRef(1));
    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
  }

  /**
   * @brief Calculate robot vertices current cost
   * @param position Robot current pose
   * @param footprint Robot vertices
   * @return Robot vertices maximum cost, if < 0, can not get the cost
   */
  double FootprintCost (const Eigen::Vector2d& position, const std::vector<Eigen::Vector2d>& footprint);

  /**
   * @brief Calculate robot vertices current cost
   * @param x Robot current x
   * @param y Robot current y
   * @param theta Robot current theta
   * @param footprint_spec Robot vertices
   * @param inscribed_radius Robot inscribed radius
   * @param circumscribed_radius Robot circumscribed radius
   * @return Robot vertices maximum cost, if < 0, can not get the cost
   */
  double FootprintCost (double x, double y, double theta,
                       const std::vector<Eigen::Vector2d>& footprint_spec,
                       double inscribed_radius = 0.0, double circumscribed_radius=0.0);

  /**
   * @brief A line max cost in cost map
   * @param x0 Line start x
   * @param x1 Line end x
   * @param y0 Line start y
   * @param y1 Line end y
   * @return Line maximum cost
   */
  double LineCost (int x0, int x1, int y0, int y1);

  /**
   * @brief A point cost in cost map
   * @param x Point x
   * @param y Point y
   * @return Point cost
   */
  double PointCost(int x, int y);

 private:
  //! Cost map
  const roborts_costmap::Costmap2D& costmap_;
};

/**
 * @brief Get robot type(Point, circular, two circles, line, polygon)
 * @tparam T Robot type class
 * @param config robot type param
 * @return Robot footprint model
 */
template <class T>
RobotFootprintModelPtr GetRobotFootprintModel (T config) {
  if (!config.robot_type(0).has_type()) {
    ROS_INFO("No robot footprint model specified. Using point-shaped model.");
    return boost::make_shared<PointRobotFootprint>();
  } else if (config.robot_type(0).type() == 0) { // point type
    ROS_INFO("Footprint model 'point' loaded");
    return boost::make_shared<PointRobotFootprint>();
  } else if (config.robot_type(0).type() == 1) {// circular
    // get radius
    double radius;
    if (!config.robot_type(0).has_radius()) {
      ROS_INFO("the circle radius doesn't exist, Using point-shaped model");
      return boost::make_shared<PointRobotFootprint>();
    }
    radius = config.robot_type(0).radius();
    ROS_INFO("Footprint model 'circular' (radius: %f m) loaded", radius);
    return boost::make_shared<CircularRobotFootprint>(radius);
  } else if (config.robot_type(0).type() == 3) { // line
    if (! config.robot_type(0).robot_vertices_size()) {
      ROS_INFO("the line vertices don't exist, Using point-shaped model");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get line coordinates
    Eigen::Vector2d line_start, line_end;
    line_start.coeffRef(0) = config.robot_type(0).robot_vertices(0).x();
    line_start.coeffRef(1) = config.robot_type(0).robot_vertices(0).y();

    line_end.coeffRef(0) = config.robot_type(0).robot_vertices(1).x();
    line_end.coeffRef(1) = config.robot_type(0).robot_vertices(1).y();

    if (config.robot_type(0).robot_vertices_size() != 2) {
      ROS_INFO("the vertices' size doesn't equal 2, Using point-shaped model");
      return boost::make_shared<PointRobotFootprint>();
    }

    ROS_INFO("Footprint model 'line' loaded");
    return boost::make_shared<LineRobotFootprint>(line_start, line_end);
  } else if (config.robot_type(0).type() == 2) {

    if (!config.robot_type(0).front_offset()|| !config.robot_type(0).front_radius()
        || !config.robot_type(0).rear_offset() || !config.robot_type(0).rear_radius() ) {
      ROS_INFO("the param is not enough to combined two circle, Using point-shaped model");
      return boost::make_shared<PointRobotFootprint>();
    }
    double front_offset, front_radius, rear_offset, rear_radius;
    front_offset = config.robot_type(0).front_offset();
    front_radius = config.robot_type(0).front_radius();
    rear_offset = config.robot_type(0).rear_offset();
    rear_radius = config.robot_type(0).rear_radius();

    ROS_INFO("Footprint model 'two circle' loaded");
    return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  } else if ( config.robot_type(0).type() == 4) {

    if (!config.robot_type(0).robot_vertices_size() ) {
      ROS_INFO("the vertices is null, Using point-shaped model");
      return boost::make_shared<PointRobotFootprint>();
    }
    Point2dContainer polygon;
    for (int j = 0; j < config.robot_type(0).robot_vertices_size(); ++j) {
      Eigen::Vector2d temp_point;
      temp_point.coeffRef(0) = config.robot_type(0).robot_vertices(j).x();
      temp_point.coeffRef(1) = config.robot_type(0).robot_vertices(j).y();
      polygon.push_back(temp_point);
    }

    ROS_INFO("Footprint model 'polygon' loaded");
    return boost::make_shared<PolygonRobotFootprint>(polygon);
  } else {
    ROS_INFO("this type doesn't exist, Using point-shaped model");
    return boost::make_shared<PointRobotFootprint>();
  }

}


} // namespace roborts_local_planner

#endif //ROBORTS_PLANNING_LOCAL_PLANNER_ROBOT_POSITION_COST_H

