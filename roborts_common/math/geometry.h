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

#ifndef ROBORTS_COMMON_GEOMETRY_H
#define ROBORTS_COMMON_GEOMETRY_H

#include <cmath>
#include <string>
#include <vector>

namespace roborts_common {

constexpr double kEpsilon = 1e-10;

class Point2D {
 public:

  constexpr Point2D(const double x, const double y) noexcept : x_(x), y_(y),  angle_(std::atan2(y_, x_)) {}

  constexpr Point2D() noexcept : Point2D(0, 0) {}

  double X() const {
    return x_;
  }

  double Y() const {
    return y_;
  }

  double Angle() const {
    return angle_;
  }

  void SetX(const double x) {
    x_ = x;
  }

  void SetY(const double y) {
    y_ = y;
  }

  void Normalize() {
      x_ = cos(angle_);
      y_ = sin(angle_);
  }

  Point2D Rotate(const double angle) const {
    double x = x_ * cos(angle) - y_ * sin(angle);
    double y = x_ * sin(angle) + y_ * cos(angle);
    return Point2D(x, y);
  }

  friend Point2D operator+(const Point2D &point_a, const Point2D &point_b) {
    double x = point_a.X() + point_b.X();
    double y = point_a.Y() + point_b.Y();
    return Point2D(x, y);
  }

  friend Point2D operator-(const Point2D &point_a, const Point2D &point_b) {
    double x = point_a.X() - point_b.X();
    double y = point_a.Y() - point_b.Y();
    return Point2D(x, y);
  };

  Point2D operator*(const double ratio) const {
    return Point2D(x_ * ratio, y_ * ratio);
  }

  Point2D &operator+=(const Point2D &rhs) {
    x_ += rhs.X();
    y_ += rhs.Y();
    return *this;
  }

  Point2D &operator-=(const Point2D &rhs) {
    x_ -= rhs.X();
    y_ -= rhs.Y();
    return *this;
  };

  Point2D &operator*=(const double ratio) {
    x_ *= ratio;
    y_ *= ratio;
    return *this;
  };

  friend bool operator==(const Point2D &point_a, const Point2D &point_b) {
    return (std::abs(point_a.X() - point_b.X()) < kEpsilon &&
        std::abs(point_a.Y() - point_b.Y()) < kEpsilon);
  };

 protected:
  double x_;
  double y_;
  double angle_;
};

class LineSegment2D {

 public:
  LineSegment2D();

  LineSegment2D(const Point2D &start, const Point2D &end) : start_(start), end_(end) {
    dx_ = end_.X() - start_.X();
    dy_ = end_.Y() - start_.Y();
    length_ = hypot(dx_, dy_);
    unit_direction_ = Point2D(dx_ / length_, dy_ / length_);
  }

  const Point2D &Start() const {
    return start_;
  }

  const Point2D &End() const {
    return end_;
  }

  const double &DiffX() const {
    return dx_;
  }

  const double &DiffY() const {
    return dy_;
  }

  const Point2D &UnitDirection() const {
    return unit_direction_;
  }

  Point2D Center() const {
    return (start_ + end_) * 0.5;
  }

 private:
  Point2D start_;
  Point2D end_;
  Point2D unit_direction_;
  double length_;
  double dx_;
  double dy_;
};

class Polygon2D {
 public:

  Polygon2D() = default;

  explicit Polygon2D(std::vector<Point2D> points) : points_(std::move(points)) {
    BuildFromPoints();
  }

  const std::vector<Point2D> &Points() const {
    return points_;
  }

  const std::vector<LineSegment2D> &Lines() const {
    return line_segments_;
  }

  int NumPoints() const {
    return num_points_;
  }

  bool IsConvex() const {
    return is_convex_;
  }

  double Area() const {
    return area_;
  }

  double MinX() const { return min_x_; }
  double MaxX() const { return max_x_; }
  double MinY() const { return min_y_; }
  double MaxY() const { return max_y_; }

 protected:
  void BuildFromPoints() {

    num_points_ = points_.size();
    line_segments_.reserve(num_points_);

    for (int i = 1; i < num_points_; ++i) {
      auto vector0 = points_[i - 1] - points_[0];
      auto vector1 = points_[i] - points_[0];
      area_ += vector0.X() * vector1.Y() - vector0.Y() * vector1.X();
    }
    if (area_ < 0) {
      std::reverse(points_.begin(), points_.end());
    }
    area_ = area_ / 2.0;

    // Construct line_segments.
    line_segments_.reserve(num_points_);
    for (int i = 0; i < num_points_; ++i) {
      line_segments_.emplace_back(points_[i], points_[Next(i)]);
    }

    //use cross judge convex
    is_convex_ = true;
    for (int i = 0; i < num_points_; ++i) {

      auto vector0 = points_[i] - points_[Prev(i)];
      auto vector1 = points_[Next(i)] - points_[Prev(i)];

      if ((vector0.X() * vector1.Y() - vector0.Y() * vector1.X()) <= -kEpsilon) {
        is_convex_ = false;
        break;
      }
    }
  }
  int Next(int index) const {
    return index >= (num_points_ - 1) ? 0 : (index + 1);
  }

  int Prev(int index) const {
    return index == 0 ? (num_points_ - 1) : (index - 1);
  }

  static bool ClipConvexHull(const LineSegment2D &line_segment,
                             std::vector<Point2D> *const points);

  std::vector<Point2D> points_;
  int num_points_ = 0;
  std::vector<LineSegment2D> line_segments_;
  bool is_convex_ = false;
  double area_ = 0.0;
  double min_x_ = 0.0;
  double max_x_ = 0.0;
  double min_y_ = 0.0;
  double max_y_ = 0.0;
};

} // namespace roborts_common
#endif //ROBORTS_COMMON_GEOMETRY_H
