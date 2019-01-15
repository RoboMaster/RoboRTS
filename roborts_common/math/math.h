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

#ifndef ROBORTS_COMMON_MATH_H
#define ROBORTS_COMMON_MATH_H

#include <iostream>
#include <vector>

#include <Eigen/Core>

#include "geometry.h"
namespace roborts_common {

inline double Cross2D (const geometry::Point2D &vector0,
                       const geometry::Point2D &vector1) {
  return vector0.X() * vector1.Y() - vector0.Y() * vector1.X();
}

inline double Cross2D (const geometry::Point2D &start_point,
                       const geometry::Point2D &point0,
                       const geometry::Point2D &point1) {
  auto vector0 = point0 - start_point;
  auto vector1 = point1 - start_point;
  return Cross2D(vector0, vector1);
}

inline double Cross2D (const Eigen::Vector2d v1,
                       const Eigen::Vector2d v2) {
  return v1.x()*v2.y() - v2.x()*v1.y();
}

inline double Cross2D(const double x0, const double y0,
                      const double x1, const double y1) {
  return x0 * y1 - x1 * y0;
}

inline bool ConvexVerify(std::vector<geometry::Point2D> &points) {

  auto num_points = points.size();
  if (num_points < 4) {
    return true;
  }

  //make sure the point range is Anti-Clock
  double area = 0;
  for (int i = 1; i < num_points; ++i) {
    area += Cross2D(points[0], points[i - 1], points[i]);
  }
  if (area < 0) {
    std::reverse(points.begin(), points.end());
  }

//use cross judge convex
  for (int i = 0; i < num_points; ++i) {

    auto prev = i == 0 ? (num_points - 1) : (i - 1);
    auto next = i >= (num_points - 1) ? 0 : (i + 1);

    if (Cross2D(points[prev], points[i], points[next]) <= 1e-10) {
      return false;
    }
  }

  return true;
}

inline double LogisticSigmoid (const double x) {
  return x / (1 + fabs(x));
}

inline double PointDistance (const double x0, const double y0,
                             const double x1, const double y1) {
  return hypot(x1 - x0, y1 - y0);
}

inline double PointDistance (const geometry::Point2D &point0,
                             const geometry::Point2D &point1) {
  return hypot((point1 - point0).X(), (point1 - point0).Y());
}

inline double PointDistance (const Eigen::Vector2d &vector0,
                             const Eigen::Vector2d &vector1) {
  return hypot(vector1.x() - vector0.x(), vector1.y() - vector0.y());
}

inline double PointToLineDistance (const double px, const double py,
                                   const double x0, const double y0,
                                   const double x1, const double y1) {
  double A = px - x0;
  double B = py - y0;
  double C = x1 - x0;
  double D = y1 - y0;

  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  double param = dot / len_sq;

  double xx, yy;

  if (param < 0)
  {
    xx = x0;
    yy = y0;
  }
  else if (param > 1)
  {
    xx = x1;
    yy = y1;
  }
  else
  {
    xx = x0 + param * C;
    yy = y0 + param * D;
  }

  return PointDistance(px, py, xx, yy);
}

inline double PointToLineDistance (const geometry::Point2D &point,
                                   const geometry::LineSegment2D &line) {
  double A = point.X() - line.Start().X();
  double B = point.Y() - line.Start().Y();
  double C = line.DiffX();
  double D = line.DiffY();

  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  double param = dot / len_sq;

  double xx, yy;

  if (param < 0)
  {
    xx = line.Start().X();
    yy = line.Start().Y();
  }
  else if (param > 1)
  {
    xx = line.End().X();
    yy = line.End().Y();
  }
  else
  {
    xx = line.Start().X() + param * C;
    yy = line.Start().Y() + param * D;
  }

  return PointDistance(point.X(), point.Y(), xx, yy);
}

inline double PointToLineDistance(const Eigen::Ref<const Eigen::Vector2d> &point,
                                                   const Eigen::Ref<const Eigen::Vector2d> &line_start,
                                                   const Eigen::Ref<const Eigen::Vector2d> &line_end) {
  Eigen::Vector2d diff = line_end - line_start;
  double sq_norm = diff.squaredNorm();

  if (sq_norm == 0){
    return (point - line_start).norm();
  }

  double u = ((point.x() - line_start.x()) * diff.coeffRef(0) + (point.y() - line_start.y()) * diff.coeffRef(1)) / sq_norm;

  if (u <= 0) {
    return (point - line_start).norm();
  } else if (u >= 1) {
    return (point - line_end).norm();
  }

  return PointDistance(point, line_start + u * diff);
}

inline bool CheckLineSegmentsIntersection2D(const Eigen::Ref<const Eigen::Vector2d> &line1_start,
                                            const Eigen::Ref<const Eigen::Vector2d> &line1_end,
                                            const Eigen::Ref<const Eigen::Vector2d> &line2_start,
                                            const Eigen::Ref<const Eigen::Vector2d> &line2_end,
                                            Eigen::Vector2d *intersection = NULL) {
  double s_numer, t_numer, denom, t;
  Eigen::Vector2d line1 = line1_end - line1_start;
  Eigen::Vector2d line2 = line2_end - line2_start;

  denom = line1.coeffRef(0) * line2.coeffRef(1) - line2.coeffRef(0) * line1.coeffRef(1);
  if (denom == 0) {
    return false;
  }
  bool denomPositive = denom > 0;

  Eigen::Vector2d aux = line1_start - line2_start;

  s_numer = line1.coeffRef(0) * aux.coeffRef(1) - line1.coeff(1) * aux.coeffRef(0);
  if ((s_numer < 0) == denomPositive) {
    return false;
  }

  t_numer = line2.coeffRef(0) * aux.coeffRef(1) - line2.coeffRef(1) * aux.coeffRef(0);
  if ((t_numer < 0) == denomPositive) {
    return false;
  }

  if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive)) {
    return false;
  }

  t = t_numer / denom;
  if (intersection) {
    *intersection = line1_start + t * line1;
  }

  return true;
}

inline bool CheckLineSegmentsIntersection2D(const geometry::LineSegment2D &line0,
                                            const geometry::LineSegment2D &line1,
                                            geometry::Point2D *intersection = NULL) {
  double s_numer, t_numer, denom, t;

  denom = line0.DiffX() * line1.DiffY() - line1.DiffX() * line0.DiffY();
  if (denom == 0) {
    return false;
  }
  bool denomPositive = denom > 0;

  geometry::LineSegment2D aux(line1.Start(), line0.Start());

  s_numer = line0.DiffX() * aux.DiffY() - line0.DiffY() * aux.DiffX();
  if ((s_numer < 0) == denomPositive) {
    return false;
  }

  t_numer = line1.DiffX() * aux.DiffY() - line1.DiffY() * aux.DiffX();
  if ((t_numer < 0) == denomPositive) {
    return false;
  }

  if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive)) {
    return false;
  }

  t = t_numer / denom;
  if (intersection) {
    *intersection = line0.Start() + geometry::Point2D(t * line0.DiffX(), t * line0.DiffY());
  }

  return true;
}

inline double DistanceSegmentToSegment2D(const Eigen::Ref<const Eigen::Vector2d> &line1_start,
                                         const Eigen::Ref<const Eigen::Vector2d> &line1_end,
                                         const Eigen::Ref<const Eigen::Vector2d> &line2_start,
                                         const Eigen::Ref<const Eigen::Vector2d> &line2_end) {

  if (CheckLineSegmentsIntersection2D(line1_start, line1_end, line2_start, line2_end)){
    return 0;
  }

  std::array<double, 4> distances;

  distances[0] = PointToLineDistance(line1_start, line2_start, line2_end);
  distances[1] = PointToLineDistance(line1_end, line2_start, line2_end);
  distances[2] = PointToLineDistance(line2_start, line1_start, line1_end);
  distances[3] = PointToLineDistance(line2_end, line1_start, line1_end);

  return *std::min_element(distances.begin(), distances.end());
}

inline double DistanceSegmentToSegment2D(const geometry::LineSegment2D &line0, const geometry::LineSegment2D &line1) {

  if (CheckLineSegmentsIntersection2D(line0, line1)) {
    return 0;
  }

  std::array<double, 4> distances;

  distances[0] = PointToLineDistance(line0.Start(), line1);
  distances[1] = PointToLineDistance(line0.End(), line1);
  distances[2] = PointToLineDistance(line1.Start(), line0);
  distances[3] = PointToLineDistance(line1.End(), line0);

  return *std::min_element(distances.begin(), distances.end());
}


inline double DistancePointToPolygon2D(const Eigen::Vector2d &point, const std::vector<Eigen::Vector2d> &vertices) {
  double dist = HUGE_VAL;

  if (vertices.size() == 1) {
    return (point - vertices.front()).norm();
  }

  for (int i = 0; i < (int) vertices.size() - 1; ++i) {

    double new_dist = PointToLineDistance(point.coeffRef(0), point.coeffRef(1), vertices.at(i).coeffRef(0), vertices.at(i).coeffRef(1),
                                          vertices.at(i + 1).coeffRef(0), vertices.at(i + 1).coeffRef(1));
    if (new_dist < dist) {
      dist = new_dist;
    }

  }

  if (vertices.size() > 2) {
  double new_dist = PointToLineDistance(point.coeffRef(0), point.coeffRef(1), vertices.back().coeffRef(0), vertices.back().coeffRef(1),
                                        vertices.front().coeffRef(0), vertices.front().coeffRef(1));
    if (new_dist < dist) {
      std::cout << "here" << std::endl;
      dist = new_dist;
    }
  }

  return dist;
}

inline double DistancePointToPolygon2D(const geometry::Point2D &point, const geometry::Polygon2D &polygon) {
  double dist = HUGE_VAL;

  if (polygon.NumPoints() == 1) {
    return PointDistance(point, polygon.Points().front());
  }

  for (int i = 0; i < (int) polygon.Lines().size() - 1; ++i) {

    double new_dist = PointToLineDistance(point, polygon.Lines().at(i));
    if (new_dist < dist) {
      dist = new_dist;
    }

  }

  return dist;
}

inline double DistanceSegmentToPolygon2D(const Eigen::Vector2d &line_start,
                                         const Eigen::Vector2d &line_end,
                                         const std::vector<Eigen::Vector2d> &vertices) {
  double dist = HUGE_VAL;

  if (vertices.size() == 1) {
    return PointToLineDistance(vertices.front(), line_start, line_end);
  }

  for (int i = 0; i < (int) vertices.size() - 1; ++i) {
    double new_dist = DistanceSegmentToSegment2D(line_start, line_end, vertices.at(i), vertices.at(i + 1));
    if (new_dist < dist) {
      dist = new_dist;
    }
  }

  if (vertices.size() > 2) {
    double new_dist =
        DistanceSegmentToSegment2D(line_start, line_end, vertices.back(), vertices.front());
    if (new_dist < dist)
      return new_dist;
  }

  return dist;
}

inline double DistanceSegmentToPolygon2D(const geometry::LineSegment2D line,
                                         const geometry::Polygon2D polygon) {
  double dist = HUGE_VAL;

  if (polygon.Points().size() == 1) {
    return PointToLineDistance(polygon.Points().front(), line);
  }

  for (int i = 0; i < (int) (polygon.Lines().size() - 1); ++i) {
    double new_dist = DistanceSegmentToSegment2D(line, polygon.Lines().at(i));
    if (new_dist < dist) {
      dist = new_dist;
    }
  }

  return dist;
}

inline double DistancePolygonToPolygon2D(const std::vector<Eigen::Vector2d> &vertices1,
                                         const std::vector<Eigen::Vector2d> &vertices2) {
  double dist = HUGE_VAL;

  if (vertices1.size() == 1) {
    return DistancePointToPolygon2D(vertices1.front(), vertices2);
  }

  for (int i = 0; i < (int) vertices1.size() - 1; ++i) {
    double new_dist = DistanceSegmentToPolygon2D(vertices1[i], vertices1[i + 1], vertices2);
    if (new_dist < dist) {
      dist = new_dist;
    }
  }

  if (vertices1.size() > 2) {
    double new_dist = DistanceSegmentToPolygon2D(vertices1.back(), vertices1.front(), vertices2);
    if (new_dist < dist) {
      return new_dist;
    }
  }

  return dist;
}

inline double DistancePolygonToPolygon2D(const geometry::Polygon2D &polygon0,
                                         const geometry::Polygon2D &polygon1) {
  double dist = HUGE_VAL;

  if (polygon0.Points().size() == 1) {
    return DistancePointToPolygon2D(polygon0.Points().front(), polygon1);
  }

  for (int i = 0; i < (int) (polygon0.Lines().size() - 1); ++i) {
    double new_dist = DistanceSegmentToPolygon2D(polygon0.Lines().at(i), polygon1);
    if (new_dist < dist) {
      dist = new_dist;
    }
  }

  return dist;
}

} // namespace roborts_common

#endif //ROBORTS_COMMON_MATH_H
