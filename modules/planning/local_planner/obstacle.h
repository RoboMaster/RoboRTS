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

#ifndef MODULES_PLANNING_LOCAL_PLANNER_OBSTACLES_H
#define MODULES_PLANNING_LOCAL_PLANNER_OBSTACLES_H

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <complex>

#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>
#include <geometry_msgs/Polygon.h>
#include "modules/planning/local_planner/distance_calculation.h"
#include "common/log.h"


namespace rrts {
namespace planning {
namespace local_planner {

class Obstacle {
 public:
  Obstacle() : dynamic_(false), centroid_velocity_(Eigen::Vector2d::Zero()) {
  }

  virtual ~Obstacle() {
  }


  virtual const Eigen::Vector2d &GetCentroid() const = 0;

  virtual std::complex<double> GetCentroidCplx() const = 0;

  virtual bool CheckCollision(const Eigen::Vector2d &position, double min_dist) const = 0;

  virtual bool CheckLineIntersection(const Eigen::Vector2d &line_start,
                                     const Eigen::Vector2d &line_end,
                                     double min_dist = 0) const = 0;


  virtual double GetMinimumDistance(const Eigen::Vector2d &position) const = 0;


  virtual double GetMinimumDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end) const = 0;


  virtual double GetMinimumDistance(const Point2dContainer &polygon) const = 0;


  virtual Eigen::Vector2d GetClosestPoint(const Eigen::Vector2d &position) const = 0;

  bool IsDynamic() const { return dynamic_; }


  void SetCentroidVelocity(const Eigen::Ref<const Eigen::Vector2d> &vel) {
    centroid_velocity_ = vel;
    dynamic_ = true;
  }


  const Eigen::Vector2d &GetCentroidVelocity() const { return centroid_velocity_; }

  virtual void ToPolygonMsg(geometry_msgs::Polygon &polygon) = 0;

 protected:

  bool dynamic_;
  Eigen::Vector2d centroid_velocity_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<Obstacle> ObstaclePtr;

typedef boost::shared_ptr<const Obstacle> ObstacleConstPtr;

typedef std::vector<ObstaclePtr> ObstContainer;

class PointObstacle : public Obstacle {
 public:

  PointObstacle() : Obstacle(), pos_(Eigen::Vector2d::Zero()) {}

  PointObstacle(const Eigen::Ref<const Eigen::Vector2d> &position) : Obstacle(), pos_(position) {}

  PointObstacle(double x, double y) : Obstacle(), pos_(Eigen::Vector2d(x, y)) {}

  virtual bool CheckCollision(const Eigen::Vector2d &point, double min_dist) const {
    return GetMinimumDistance(point) < min_dist;
  }

  virtual bool CheckLineIntersection(const Eigen::Vector2d &line_start,
                                     const Eigen::Vector2d &line_end,
                                     double min_dist = 0) const {

    Eigen::Vector2d a = line_end - line_start;
    Eigen::Vector2d b = pos_ - line_start;

    double t = a.dot(b) / a.dot(a);
    if (t < 0) t = 0;
    else if (t > 1) t = 1;
    Eigen::Vector2d nearest_point = line_start + a * t;

    return CheckCollision(nearest_point, min_dist);
  }

  virtual double GetMinimumDistance(const Eigen::Vector2d &position) const {
    return (position - pos_).norm();
  }

  virtual double GetMinimumDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end) const {
    return DistancePointToSegment2D(pos_, line_start, line_end);
  }

  virtual double GetMinimumDistance(const Point2dContainer &polygon) const {
    return DistancePointToPolygon2D(pos_, polygon);
  }

  virtual Eigen::Vector2d GetClosestPoint(const Eigen::Vector2d &position) const {
    return pos_;
  }

  virtual const Eigen::Vector2d &GetCentroid() const {
    return pos_;
  }

  virtual std::complex<double> GetCentroidCplx() const {
    return std::complex<double>(pos_[0], pos_[1]);
  }

  const Eigen::Vector2d &Position() const {
    return pos_;
  }
  Eigen::Vector2d &Position() {
    return pos_;
  }

  virtual void ToPolygonMsg(geometry_msgs::Polygon &polygon) {
    polygon.points.resize(1);
    polygon.points.front().x = pos_.x();
    polygon.points.front().y = pos_.y();
    polygon.points.front().z = 0;
  }

 protected:

  Eigen::Vector2d pos_;


 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class LineObstacle : public Obstacle {
 public:
  typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > VertexContainer;

  LineObstacle() : Obstacle() {
    start_.setZero();
    end_.setZero();
    centroid_.setZero();
  }

  LineObstacle(const Eigen::Ref<const Eigen::Vector2d> &line_start, const Eigen::Ref<const Eigen::Vector2d> &line_end)
      : Obstacle(), start_(line_start), end_(line_end) {
    CalcCentroid();
  }

  LineObstacle(double x1, double y1, double x2, double y2) : Obstacle() {
    start_.x() = x1;
    start_.y() = y1;
    end_.x() = x2;
    end_.y() = y2;
    CalcCentroid();
  }

  virtual bool CheckCollision(const Eigen::Vector2d &point, double min_dist) const {
    return GetMinimumDistance(point) <= min_dist;
  }

  virtual bool CheckLineIntersection(const Eigen::Vector2d &line_start,
                                     const Eigen::Vector2d &line_end,
                                     double min_dist = 0) const {
    return CheckLineSegmentsIntersection2D(line_start, line_end, start_, end_);
  }

  virtual double GetMinimumDistance(const Eigen::Vector2d &position) const {
    return DistancePointToSegment2D(position, start_, end_);
  }

  virtual double GetMinimumDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end) const {
    return DistanceSegmentToSegment2D(start_, end_, line_start, line_end);
  }

  virtual double GetMinimumDistance(const Point2dContainer &polygon) const {
    return DistanceSegmentToPolygon2D(start_, end_, polygon);
  }

  virtual Eigen::Vector2d GetClosestPoint(const Eigen::Vector2d &position) const {
    return ClosestPointOnLineSegment2D(position, start_, end_);
  }

  virtual const Eigen::Vector2d &GetCentroid() const {
    return centroid_;
  }

  virtual std::complex<double> GetCentroidCplx() const {
    return std::complex<double>(centroid_.x(), centroid_.y());
  }

  const Eigen::Vector2d &Start() const {
    return start_;
  }
  void SetStart(const Eigen::Ref<const Eigen::Vector2d> &start) {
    start_ = start;
    CalcCentroid();
  }
  const Eigen::Vector2d &End() const {
    return end_;
  }
  void SetEnd(const Eigen::Ref<const Eigen::Vector2d> &end) {
    end_ = end;
    CalcCentroid();
  }

  virtual void ToPolygonMsg(geometry_msgs::Polygon &polygon) {
    polygon.points.resize(2);
    polygon.points.front().x = start_.x();
    polygon.points.front().y = start_.y();

    polygon.points.back().x = end_.x();
    polygon.points.back().y = end_.y();
    polygon.points.back().z = polygon.points.front().z = 0;
  }

 protected:
  void CalcCentroid() {
    centroid_ = 0.5 * (start_ + end_);
  }

 private:
  Eigen::Vector2d start_;
  Eigen::Vector2d end_;

  Eigen::Vector2d centroid_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class PolygonObstacle : public Obstacle {
 public:

  PolygonObstacle() : Obstacle(), finalized_(false) {
    centroid_.setConstant(NAN);
  }

  PolygonObstacle(const Point2dContainer &vertices) : Obstacle(), vertices_(vertices) {
    FinalizePolygon();
  }

  virtual bool CheckCollision(const Eigen::Vector2d &point, double min_dist) const {
    if (NoVertices() == 2)
      return GetMinimumDistance(point) <= min_dist;

    int i, j;
    bool c = false;
    for (i = 0, j = NoVertices() - 1; i < NoVertices(); j = i++) {
      if (((vertices_.at(i).y() > point.y()) != (vertices_.at(j).y() > point.y())) &&
          (point.x() < (vertices_.at(j).x() - vertices_.at(i).x()) * (point.y() - vertices_.at(i).y())
              / (vertices_.at(j).y() - vertices_.at(i).y()) + vertices_.at(i).x())) {
        c = !c;
      }
    }
    if (c > 0) {
      return true;
    }

    return min_dist == 0 ? false : GetMinimumDistance(point) < min_dist;
  }

  virtual bool CheckLineIntersection(const Eigen::Vector2d &line_start,
                                     const Eigen::Vector2d &line_end,
                                     double min_dist = 0) const;

  virtual double GetMinimumDistance(const Eigen::Vector2d &position) const {
    return DistancePointToPolygon2D(position, vertices_);
  }

  virtual double GetMinimumDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end) const {
    return DistanceSegmentToPolygon2D(line_start, line_end, vertices_);
  }

  virtual double GetMinimumDistance(const Point2dContainer &polygon) const {
    return DistancePolygonToPolygon2D(polygon, vertices_);
  }

  virtual Eigen::Vector2d GetClosestPoint(const Eigen::Vector2d &position) const;

  virtual const Eigen::Vector2d &GetCentroid() const {
    LOG_ASSERT(finalized_) << "Finalize the polygon after all vertices are added.";
    return centroid_;
  }

  virtual std::complex<double> GetCentroidCplx() const {
    LOG_ASSERT(finalized_) << "Finalize the polygon after all vertices are added.";
    return std::complex<double>(centroid_.coeffRef(0), centroid_.coeffRef(1));
  }

  virtual void ToPolygonMsg(geometry_msgs::Polygon &polygon);

  const Point2dContainer &Vertices() const {
    return vertices_;
  }

  Point2dContainer &Vertices() {
    return vertices_;
  }

  void PushBackVertex(const Eigen::Ref<const Eigen::Vector2d> &vertex) {
    vertices_.push_back(vertex);
    finalized_ = false;
  }

  void FinalizePolygon() {
    FixPolygonClosure();
    CalcCentroid();
    finalized_ = true;
  }

  void ClearVertices() {
    vertices_.clear();
    finalized_ = false;
  }

  int NoVertices() const {
    return (int) vertices_.size();
  }

 protected:

  void FixPolygonClosure();

  void CalcCentroid();


  Point2dContainer vertices_;
  Eigen::Vector2d centroid_;

  bool finalized_;



 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace local_planner
} // namespace planning
} // namespace rrts
#endif // MODULES_PLANNING_LOCAL_PLANNER_OBSTACLES_H
