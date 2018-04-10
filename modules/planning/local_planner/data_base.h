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

#ifndef MODULES_PLANNING_LOCAL_PLANNER_DATA_CONVERT_BASE_H
#define MODULES_PLANNING_LOCAL_PLANNER_DATA_CONVERT_BASE_H

#include <iostream>

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <g2o/stuff/misc.h>

namespace rrts {
namespace planning {
namespace local_planner {
class DataBase {
 public:
  DataBase () {
    SetZero();
  }

  DataBase (const Eigen::Ref<const Eigen::Vector2d>& position, double theta) {
    position_ = position;
    theta_ = theta;
  }

  ~DataBase() {}

  void SetPosition(Eigen::Vector2d position) {
    position_ = position;
  }

  Eigen::Vector2d& GetPosition () {
    return  position_;
  }

  const Eigen::Vector2d& GetPosition () const {
    return position_;
  }

  double& GetTheta() {
    return theta_;
  }

  void SetTheta(double theta) {
    theta_ = theta;
  }

  const double& GetTheta() const {
    return theta_;
  }

  void Plus (const double* pose_array) {
    position_.coeffRef(0) += pose_array[0];
    position_.coeffRef(1) += pose_array[1];
    theta_ = g2o::normalize_theta( theta_ + pose_array[2] );
  }

  void AverageInPlace (const DataBase& pose1, const DataBase& pose2) {
    position_ = (pose1.position_ + pose2.position_)/2;
    theta_ = g2o::average_angle(pose1.theta_, pose2.theta_);
  }

  void RotateGlobal (double angle, bool adjust_theta=true) {
    double new_x = std::cos(angle)*position_.x() - std::sin(angle)*position_.y();
    double new_y = std::sin(angle)*position_.x() + std::cos(angle)*position_.y();
    position_.x() = new_x;
    position_.y() = new_y;
    if (adjust_theta)
      theta_ = g2o::normalize_theta(theta_+angle);
  }

  Eigen::Vector2d OrientationUnitVec() const {
    return Eigen::Vector2d(std::cos(theta_), std::sin(theta_));
  }

  void SetZero() {
    position_.setZero();
    theta_ = 0;
  }

  DataBase& operator= ( const DataBase& rhs ) {
    if (&rhs != this) {
      position_ = rhs.position_;
      theta_ = rhs.theta_;
    }
    return *this;
  }

  DataBase& operator+= (const DataBase& rhs) {
    position_ += rhs.position_;
    theta_ = g2o::normalize_theta(theta_ + rhs.theta_);
    return *this;
  }

  friend DataBase operator+ (DataBase lhs, const DataBase& rhs) {
    return lhs += rhs;
  }

  DataBase& operator-= (const DataBase& rhs) {
    theta_ -= rhs.theta_;
    theta_ = g2o::normalize_theta(theta_ - rhs.theta_);
    return *this;
  }

  friend DataBase operator- (DataBase lhs, const DataBase& rhs) {
    return lhs -= rhs;
  }

  friend DataBase operator* (DataBase pose, double scalar) {
    pose.position_ *= scalar;
    pose.theta_ *= scalar;
    return pose;
  }



 private:
  Eigen::Vector2d position_;
  double theta_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}; // class TebDataBase


} // namespace local_planner
} // namespace planning
} // namespace rrts


#endif //MODULES_PLANNING_LOCAL_PLANNER_DATA_CONVERT_BASE_H