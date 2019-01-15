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
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_TEB_ACCELERATION_EAGE_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_TEB_ACCELERATION_EAGE_H

#include <geometry_msgs/Twist.h>

#include "local_planner/utility_tool.h"

#include "timed_elastic_band/teb_vertex_pose.h"
#include "timed_elastic_band/teb_vertex_timediff.h"
#include "timed_elastic_band/teb_penalties.h"
#include "timed_elastic_band/teb_base_eage.h"

namespace roborts_local_planner {

class AccelerationEdge : public TebMultiEdgeBase<2, double> {
 public:

  AccelerationEdge() {
    this->resize(5);
  }

  void computeError() {
    const TebVertexPose *pose1 = static_cast<const TebVertexPose *>(_vertices[0]);
    const TebVertexPose *pose2 = static_cast<const TebVertexPose *>(_vertices[1]);
    const TebVertexPose *pose3 = static_cast<const TebVertexPose *>(_vertices[2]);
    const TebVertexTimeDiff *dt1 = static_cast<const TebVertexTimeDiff *>(_vertices[3]);
    const TebVertexTimeDiff *dt2 = static_cast<const TebVertexTimeDiff *>(_vertices[4]);

    const Eigen::Vector2d diff1 = pose2->GetPose().GetPosition() - pose1->GetPose().GetPosition();
    const Eigen::Vector2d diff2 = pose3->GetPose().GetPosition() - pose2->GetPose().GetPosition();

    double dist1 = diff1.norm();
    double dist2 = diff2.norm();
    const double angle_diff1 = g2o::normalize_theta(pose2->GetPose().GetTheta() - pose1->GetPose().GetTheta());
    const double angle_diff2 = g2o::normalize_theta(pose3->GetPose().GetTheta() - pose2->GetPose().GetTheta());

    if (config_param_->trajectory_opt().exact_arc_length()) {
      if (angle_diff1 != 0) {
        const double radius = dist1 / (2 * sin(angle_diff1 / 2));
        dist1 = fabs(angle_diff1 * radius);
      }
      if (angle_diff2 != 0) {
        const double radius = dist2 / (2 * sin(angle_diff2 / 2));
        dist2 = fabs(angle_diff2 * radius);
      }
    }

    double vel1 = dist1 / dt1->GetDiffTime();
    double vel2 = dist2 / dt2->GetDiffTime();


    vel1 *= LogisticSigmoid(100 * (diff1.x() * cos(pose1->GetPose().GetTheta()) + diff1.y() * sin(pose1->GetPose().GetTheta())));
    vel2 *= LogisticSigmoid(100 * (diff2.x() * cos(pose2->GetPose().GetTheta()) + diff2.y() * sin(pose2->GetPose().GetTheta())));

    const double acc_lin = (vel2 - vel1) * 2 / (dt1->GetDiffTime() + dt2->GetDiffTime());

    _error[0] = PenaltyBoundToInterval(acc_lin, config_param_->kinematics_opt().acc_lim_x(),
                                       config_param_->optimize_info().penalty_epsilon());

    const double omega1 = angle_diff1 / dt1->GetDiffTime();
    const double omega2 = angle_diff2 / dt2->GetDiffTime();
    const double acc_rot = (omega2 - omega1) * 2 / (dt1->GetDiffTime() + dt2->GetDiffTime());

    _error[1] = PenaltyBoundToInterval(acc_rot, config_param_->kinematics_opt().acc_lim_theta(),
                                       config_param_->optimize_info().penalty_epsilon());
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

class AccelerationStartEdge : public TebMultiEdgeBase<2, const geometry_msgs::Twist *> {
 public:

  AccelerationStartEdge() {
    _measurement = NULL;
    this->resize(3);
  }

  void computeError() {
    const TebVertexPose *pose1 = static_cast<const TebVertexPose *>(_vertices[0]);
    const TebVertexPose *pose2 = static_cast<const TebVertexPose *>(_vertices[1]);
    const TebVertexTimeDiff *dt = static_cast<const TebVertexTimeDiff *>(_vertices[2]);

    const Eigen::Vector2d diff = pose2->GetPose().GetPosition() - pose1->GetPose().GetPosition();
    double dist = diff.norm();
    const double angle_diff = g2o::normalize_theta(pose2->GetPose().GetTheta() - pose1->GetPose().GetTheta());
    if (config_param_->trajectory_opt().exact_arc_length() && angle_diff != 0) {
      const double radius = dist / (2 * sin(angle_diff / 2));
      dist = fabs(angle_diff * radius);
    }

    const double vel1 = _measurement->linear.x;
    double vel2 = dist / dt->GetDiffTime();

    vel2 *= LogisticSigmoid(100 * (diff.x() * cos(pose1->GetPose().GetTheta()) + diff.y() * sin(pose1->GetPose().GetTheta())));

    const double acc_lin = (vel2 - vel1) / dt->GetDiffTime();

    _error[0] = PenaltyBoundToInterval(acc_lin, config_param_->kinematics_opt().acc_lim_x(),
                                       config_param_->optimize_info().penalty_epsilon());

    const double omega1 = _measurement->angular.z;
    const double omega2 = angle_diff / dt->GetDiffTime();
    const double acc_rot = (omega2 - omega1) / dt->GetDiffTime();

    _error[1] = PenaltyBoundToInterval(acc_rot, config_param_->kinematics_opt().acc_lim_theta(),
                                       config_param_->optimize_info().penalty_epsilon());
  }

  void SetInitialVelocity(const geometry_msgs::Twist &vel_start) {
    _measurement = &vel_start;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class AccelerationGoalEdge : public TebMultiEdgeBase<2, const geometry_msgs::Twist *> {
 public:

  AccelerationGoalEdge() {
    _measurement = NULL;
    this->resize(3);
  }

  void computeError() {
    const TebVertexPose *pose_pre_goal = static_cast<const TebVertexPose *>(_vertices[0]);
    const TebVertexPose *pose_goal = static_cast<const TebVertexPose *>(_vertices[1]);
    const TebVertexTimeDiff *dt = static_cast<const TebVertexTimeDiff *>(_vertices[2]);

    const Eigen::Vector2d diff = pose_goal->GetPose().GetPosition() - pose_pre_goal->GetPose().GetPosition();
    double dist = diff.norm();
    const double angle_diff = g2o::normalize_theta(pose_goal->GetPose().GetTheta() - pose_pre_goal->GetPose().GetTheta());
    if (config_param_->trajectory_opt().exact_arc_length() && angle_diff != 0) {
      double radius = dist / (2 * sin(angle_diff / 2));
      dist = fabs(angle_diff * radius);
    }

    double vel1 = dist / dt->GetDiffTime();
    const double vel2 = _measurement->linear.x;

    vel1 *= LogisticSigmoid(100 * (diff.x() * cos(pose_pre_goal->GetPose().GetTheta()) + diff.y() * sin(pose_pre_goal->GetPose().GetTheta())));

    const double acc_lin = (vel2 - vel1) / dt->GetDiffTime();

    _error[0] = PenaltyBoundToInterval(acc_lin, config_param_->kinematics_opt().acc_lim_x(),
                                       config_param_->optimize_info().penalty_epsilon());

    const double omega1 = angle_diff / dt->GetDiffTime();
    const double omega2 = _measurement->angular.z;
    const double acc_rot = (omega2 - omega1) / dt->GetDiffTime();

    _error[1] = PenaltyBoundToInterval(acc_rot, config_param_->kinematics_opt().acc_lim_theta(),
                                       config_param_->optimize_info().penalty_epsilon());

  }

  void SetGoalVelocity(const geometry_msgs::Twist &vel_goal) {
    _measurement = &vel_goal;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class AccelerationHolonomicEdge : public TebMultiEdgeBase<3, double> {
 public:

  AccelerationHolonomicEdge() {
    this->resize(5);
  }

  void computeError() {
    const TebVertexPose *pose1 = static_cast<const TebVertexPose *>(_vertices[0]);
    const TebVertexPose *pose2 = static_cast<const TebVertexPose *>(_vertices[1]);
    const TebVertexPose *pose3 = static_cast<const TebVertexPose *>(_vertices[2]);
    const TebVertexTimeDiff *dt1 = static_cast<const TebVertexTimeDiff *>(_vertices[3]);
    const TebVertexTimeDiff *dt2 = static_cast<const TebVertexTimeDiff *>(_vertices[4]);

    Eigen::Vector2d diff1 = pose2->GetPose().GetPosition() - pose1->GetPose().GetPosition();
    Eigen::Vector2d diff2 = pose3->GetPose().GetPosition() - pose2->GetPose().GetPosition();

    double cos_theta1 = std::cos(pose1->GetPose().GetTheta());
    double sin_theta1 = std::sin(pose1->GetPose().GetTheta());
    double cos_theta2 = std::cos(pose2->GetPose().GetTheta());
    double sin_theta2 = std::sin(pose2->GetPose().GetTheta());

    double p1_dx = cos_theta1 * diff1.x() + sin_theta1 * diff1.y();
    double p1_dy = -sin_theta1 * diff1.x() + cos_theta1 * diff1.y();

    double p2_dx = cos_theta2 * diff2.x() + sin_theta2 * diff2.y();
    double p2_dy = -sin_theta2 * diff2.x() + cos_theta2 * diff2.y();

    double vel1_x = p1_dx / dt1->GetDiffTime();
    double vel1_y = p1_dy / dt1->GetDiffTime();
    double vel2_x = p2_dx / dt2->GetDiffTime();
    double vel2_y = p2_dy / dt2->GetDiffTime();

    double dt12 = dt1->GetDiffTime() + dt2->GetDiffTime();

    double acc_x = (vel2_x - vel1_x) * 2 / dt12;
    double acc_y = (vel2_y - vel1_y) * 2 / dt12;

    _error[0] = PenaltyBoundToInterval(acc_x, config_param_->kinematics_opt().acc_lim_x(),
                                       config_param_->optimize_info().penalty_epsilon());
    _error[1] = PenaltyBoundToInterval(acc_y, config_param_->kinematics_opt().acc_lim_y(),
                                       config_param_->optimize_info().penalty_epsilon());

    double omega1 = g2o::normalize_theta(pose2->GetPose().GetTheta() - pose1->GetPose().GetTheta()) / dt1->GetDiffTime();
    double omega2 = g2o::normalize_theta(pose3->GetPose().GetTheta() - pose2->GetPose().GetTheta()) / dt2->GetDiffTime();
    double acc_rot = (omega2 - omega1) * 2 / dt12;

    _error[2] = PenaltyBoundToInterval(acc_rot, config_param_->kinematics_opt().acc_lim_theta(),
                                       config_param_->optimize_info().penalty_epsilon());

  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

class AccelerationHolonomicStartEdge : public TebMultiEdgeBase<3, const geometry_msgs::Twist *> {
 public:

  AccelerationHolonomicStartEdge() {
    this->resize(3);
    _measurement = NULL;
  }

  void computeError() {
    const TebVertexPose *pose1 = static_cast<const TebVertexPose *>(_vertices[0]);
    const TebVertexPose *pose2 = static_cast<const TebVertexPose *>(_vertices[1]);
    const TebVertexTimeDiff *dt = static_cast<const TebVertexTimeDiff *>(_vertices[2]);

    Eigen::Vector2d diff = pose2->GetPose().GetPosition() - pose1->GetPose().GetPosition();

    double cos_theta1 = std::cos(pose1->GetPose().GetTheta());
    double sin_theta1 = std::sin(pose1->GetPose().GetTheta());

    double p1_dx = cos_theta1 * diff.x() + sin_theta1 * diff.y();
    double p1_dy = -sin_theta1 * diff.x() + cos_theta1 * diff.y();

    double vel1_x = _measurement->linear.x;
    double vel1_y = _measurement->linear.y;
    double vel2_x = p1_dx / dt->GetDiffTime();
    double vel2_y = p1_dy / dt->GetDiffTime();

    double acc_lin_x = (vel2_x - vel1_x) / dt->GetDiffTime();
    double acc_lin_y = (vel2_y - vel1_y) / dt->GetDiffTime();

    _error[0] = PenaltyBoundToInterval(acc_lin_x, config_param_->kinematics_opt().acc_lim_x(),
                                       config_param_->optimize_info().penalty_epsilon());
    _error[1] = PenaltyBoundToInterval(acc_lin_y, config_param_->kinematics_opt().acc_lim_y(),
                                       config_param_->optimize_info().penalty_epsilon());

    double omega1 = _measurement->angular.z;
    double omega2 = g2o::normalize_theta(pose2->GetPose().GetTheta() - pose1->GetPose().GetTheta()) / dt->GetDiffTime();
    double acc_rot = (omega2 - omega1) / dt->GetDiffTime();

    _error[2] = PenaltyBoundToInterval(acc_rot, config_param_->kinematics_opt().acc_lim_theta(),
                                       config_param_->optimize_info().penalty_epsilon());

  }

  void setInitialVelocity(const geometry_msgs::Twist &vel_start) {
    _measurement = &vel_start;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class AccelerationHolonomicGoalEdge : public TebMultiEdgeBase<3, const geometry_msgs::Twist *> {
 public:

  AccelerationHolonomicGoalEdge() {
    _measurement = NULL;
    this->resize(3);
  }

  void computeError() {

    const TebVertexPose *pose_pre_goal = static_cast<const TebVertexPose *>(_vertices[0]);
    const TebVertexPose *pose_goal = static_cast<const TebVertexPose *>(_vertices[1]);
    const TebVertexTimeDiff *dt = static_cast<const TebVertexTimeDiff *>(_vertices[2]);

    Eigen::Vector2d diff = pose_goal->GetPose().GetPosition() - pose_pre_goal->GetPose().GetPosition();

    double cos_theta1 = std::cos(pose_pre_goal->GetPose().GetTheta());
    double sin_theta1 = std::sin(pose_pre_goal->GetPose().GetTheta());

    double p1_dx = cos_theta1 * diff.x() + sin_theta1 * diff.y();
    double p1_dy = -sin_theta1 * diff.x() + cos_theta1 * diff.y();

    double vel1_x = p1_dx / dt->GetDiffTime();
    double vel1_y = p1_dy / dt->GetDiffTime();
    double vel2_x = _measurement->linear.x;
    double vel2_y = _measurement->linear.y;

    double acc_lin_x = (vel2_x - vel1_x) / dt->GetDiffTime();
    double acc_lin_y = (vel2_y - vel1_y) / dt->GetDiffTime();

    _error[0] = PenaltyBoundToInterval(acc_lin_x, config_param_->kinematics_opt().acc_lim_x(),
                                       config_param_->optimize_info().penalty_epsilon());
    _error[1] = PenaltyBoundToInterval(acc_lin_x, config_param_->kinematics_opt().acc_lim_y(),
                                       config_param_->optimize_info().penalty_epsilon());

    double omega1 = g2o::normalize_theta(pose_goal->GetPose().GetTheta() - pose_pre_goal->GetPose().GetTheta()) / dt->GetDiffTime();
    double omega2 = _measurement->angular.z;
    double acc_rot = (omega2 - omega1) / dt->GetDiffTime();

    _error[2] = PenaltyBoundToInterval(acc_rot, config_param_->kinematics_opt().acc_lim_theta(),
                                       config_param_->optimize_info().penalty_epsilon());
  }

  void SetGoalVelocity(const geometry_msgs::Twist &vel_goal) {
    _measurement = &vel_goal;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // roborts_local_planner
#endif // ROBORTS_PLANNING_LOCAL_PLANNER_TEB_ACCELERATION_EAGE_H_
