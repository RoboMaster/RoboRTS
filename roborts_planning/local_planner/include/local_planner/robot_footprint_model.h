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

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_ROBOT_FOOTPRINT_MODEL_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_ROBOT_FOOTPRINT_MODEL_H

#include "local_planner/data_base.h"
#include "local_planner/obstacle.h"
#include "local_planner/distance_calculation.h"

namespace roborts_local_planner {

/**
 * @brief A class to describe robot footprint
 */
class BaseRobotFootprintModel {
 public:

  /**
   * @brief Constructor
   */
  BaseRobotFootprintModel() {
  }

  /**
   * @brief Destructor
   */
  virtual ~BaseRobotFootprintModel() {
  }

  /**
   * @brief Calculate distance
   * @param current_pose Current robot pose
   * @param obstacle Obstacle want to calculate
   * @return Distance from current pose to obstalce
   */
  virtual double CalculateDistance(const DataBase &current_pose, const Obstacle *obstacle) const = 0;

  /**
   * @brief Get robot inscribed radius
   * @return Inscribed radius
   */
  virtual double GetInscribedRadius() = 0;

 public:
  //! make aligned memory
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<BaseRobotFootprintModel> RobotFootprintModelPtr;

typedef boost::shared_ptr<const BaseRobotFootprintModel> RobotFootprintModelConstPtr;

/**
 * @brief Point footprint
 */
class PointRobotFootprint : public BaseRobotFootprintModel {
 public:

  PointRobotFootprint() {}

  virtual ~PointRobotFootprint() {}

  virtual double CalculateDistance(const DataBase &current_pose, const Obstacle *obstacle) const {
    return obstacle->GetMinimumDistance(current_pose.GetPosition());
  }

  virtual double GetInscribedRadius() { return 0.0; }

};

/**
 * @brief Circular footprint
 */
class CircularRobotFootprint : public BaseRobotFootprintModel {
 public:

  CircularRobotFootprint(double radius) : radius_(radius) {}

  virtual ~CircularRobotFootprint() {}

  void SetRadius(double radius) { radius_ = radius; }

  virtual double CalculateDistance(const DataBase &current_pose, const Obstacle *obstacle) const {
    return obstacle->GetMinimumDistance(current_pose.GetPosition()) - radius_;
  }


  virtual double GetInscribedRadius() { return radius_; }

 private:

  double radius_;
};

/**
 * @brief Two circles footprint
 */
class TwoCirclesRobotFootprint : public BaseRobotFootprintModel {
 public:

  TwoCirclesRobotFootprint(double front_offset, double front_radius, double rear_offset, double rear_radius)
      : front_offset_(front_offset),
        front_radius_(front_radius),
        rear_offset_(rear_offset),
        rear_radius_(rear_radius) {}

  virtual ~TwoCirclesRobotFootprint() {}

  void SetParameters(double front_offset, double front_radius, double rear_offset, double rear_radius) {
    front_offset_ = front_offset;
    front_radius_ = front_radius;
    rear_offset_ = rear_offset;
    rear_radius_ = rear_radius;
  }

  virtual double CalculateDistance(const DataBase &current_pose, const Obstacle *obstacle) const {
    Eigen::Vector2d dir = current_pose.OrientationUnitVec();
    double dist_front = obstacle->GetMinimumDistance(current_pose.GetPosition() + front_offset_ * dir) - front_radius_;
    double dist_rear = obstacle->GetMinimumDistance(current_pose.GetPosition() - rear_offset_ * dir) - rear_radius_;
    return std::min(dist_front, dist_rear);
  }

  virtual double GetInscribedRadius() {
    double min_longitudinal = std::min(rear_offset_ + rear_radius_, front_offset_ + front_radius_);
    double min_lateral = std::min(rear_radius_, front_radius_);
    return std::min(min_longitudinal, min_lateral);
  }

 private:

  double front_offset_;
  double front_radius_;
  double rear_offset_;
  double rear_radius_;

};

/**
 * @brief Line footprint
 */
class LineRobotFootprint : public BaseRobotFootprintModel {
 public:

  LineRobotFootprint(const geometry_msgs::Point &line_start, const geometry_msgs::Point &line_end) {
    SetLine(line_start, line_end);
  }

  LineRobotFootprint(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end) {
    SetLine(line_start, line_end);
  }

  virtual ~LineRobotFootprint() {}

  void SetLine(const geometry_msgs::Point &line_start, const geometry_msgs::Point &line_end) {
    line_start_.x() = line_start.x;
    line_start_.y() = line_start.y;
    line_end_.x() = line_end.x;
    line_end_.y() = line_end.y;
  }

  void SetLine(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end) {
    line_start_ = line_start;
    line_end_ = line_end;
  }

  virtual double CalculateDistance(const DataBase &current_pose, const Obstacle *obstacle) const {
    // here we are doing the transformation into the world frame manually
    double cos_th = std::cos(current_pose.GetTheta());
    double sin_th = std::sin(current_pose.GetTheta());
    Eigen::Vector2d line_start_world;
    line_start_world.x() = current_pose.GetPosition().coeffRef(0) + cos_th * line_start_.x() - sin_th * line_start_.y();
    line_start_world.y() = current_pose.GetPosition().coeffRef(1) + sin_th * line_start_.x() + cos_th * line_start_.y();
    Eigen::Vector2d line_end_world;
    line_end_world.x() = current_pose.GetPosition().coeffRef(0)  + cos_th * line_end_.x() - sin_th * line_end_.y();
    line_end_world.y() = current_pose.GetPosition().coeffRef(1) + sin_th * line_end_.x() + cos_th * line_end_.y();
    return obstacle->GetMinimumDistance(line_start_world, line_end_world);
  }

  virtual double GetInscribedRadius() {
    return 0.0; // lateral distance = 0.0
  }

 private:

  Eigen::Vector2d line_start_;
  Eigen::Vector2d line_end_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

/**
 * @brief Polygon footprint
 */
class PolygonRobotFootprint : public BaseRobotFootprintModel {
 public:

  PolygonRobotFootprint(const Point2dContainer &vertices) : vertices_(vertices) {}

  virtual ~PolygonRobotFootprint() {}

  void SetVertices(const Point2dContainer &vertices) { vertices_ = vertices; }

  virtual double CalculateDistance(const DataBase &current_pose, const Obstacle *obstacle) const {
    // here we are doing the transformation into the world frame manually
    double cos_th = std::cos(current_pose.GetTheta());
    double sin_th = std::sin(current_pose.GetTheta());

    Point2dContainer polygon_world(vertices_.size());
    for (std::size_t i = 0; i < vertices_.size(); ++i) {
      polygon_world[i].x() = current_pose.GetPosition().coeffRef(0) + cos_th * vertices_[i].x() - sin_th * vertices_[i].y();
      polygon_world[i].y() = current_pose.GetPosition().coeffRef(1) + sin_th * vertices_[i].x() + cos_th * vertices_[i].y();

    }
    return obstacle->GetMinimumDistance(polygon_world);
  }

  virtual double GetInscribedRadius() {
    double min_dist = std::numeric_limits<double>::max();
    Eigen::Vector2d center(0.0, 0.0);

    if (vertices_.size() <= 2)
      return 0.0;

    for (int i = 0; i < (int) vertices_.size() - 1; ++i) {
      // compute distance from the robot center point to the first vertex
      double vertex_dist = vertices_[i].norm();
      double edge_dist = DistancePointToSegment2D(center, vertices_[i], vertices_[i + 1]);
      min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    }

    // we also need to check the last vertex and the first vertex
    double vertex_dist = vertices_.back().norm();
    double edge_dist = DistancePointToSegment2D(center, vertices_.back(), vertices_.front());
    return std::min(min_dist, std::min(vertex_dist, edge_dist));
  }

 private:

  Point2dContainer vertices_;

};

} // namespace roborts_local_planner

#endif /* ROBOT_FOOTPRINT_MODEL_H */
