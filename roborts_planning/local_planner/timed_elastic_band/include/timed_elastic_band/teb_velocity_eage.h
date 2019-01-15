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

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_TEB_VELOCITY_EDGE_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_TEB_VELOCITY_EDGE_H

#include <iostream>


#include "local_planner/utility_tool.h"

#include "timed_elastic_band/teb_vertex_pose.h"
#include "timed_elastic_band/teb_vertex_timediff.h"
#include "timed_elastic_band/teb_base_eage.h"
#include "timed_elastic_band/teb_penalties.h"

namespace roborts_local_planner {

class VelocityEdge : public TebMultiEdgeBase<2, double> {

 public:
  VelocityEdge() {
    this->resize(3);
  }

  void computeError() {
    const TebVertexPose *conf1 = static_cast<const TebVertexPose *>(_vertices[0]);
    const TebVertexPose *conf2 = static_cast<const TebVertexPose *>(_vertices[1]);
    const TebVertexTimeDiff *deltaT = static_cast<const TebVertexTimeDiff *>(_vertices[2]);

    const Eigen::Vector2d deltaS = conf2->estimate().GetPosition() - conf1->estimate().GetPosition();

    double dist = deltaS.norm();
    const double angle_diff = g2o::normalize_theta(conf2->GetPose().GetTheta() - conf1->GetPose().GetTheta());
    if (config_param_->trajectory_opt().exact_arc_length() && angle_diff != 0) {
      double radius = dist / (2 * sin(angle_diff / 2));
      dist = fabs(angle_diff * radius);
    }
    double vel = dist / deltaT->estimate();

    vel *= LogisticSigmoid(100 * (deltaS.x() * cos(conf1->GetPose().GetTheta())
        + deltaS.y() * sin(conf1->GetPose().GetTheta())));

    const double omega = angle_diff / deltaT->estimate();

    _error[0] = PenaltyBoundToInterval(vel, -config_param_->kinematics_opt().max_vel_x_backwards(),
                                       config_param_->kinematics_opt().max_vel_x(), config_param_->optimize_info().penalty_epsilon());
    _error[1] = PenaltyBoundToInterval(omega, config_param_->kinematics_opt().max_vel_theta(),
                                       config_param_->optimize_info().penalty_epsilon());
  }

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

class VelocityHolonomicEdge : public TebMultiEdgeBase<3, double> {
 public:

  VelocityHolonomicEdge() {
    this->resize(3);
  }

  void computeError() {

    const TebVertexPose *conf1 = static_cast<const TebVertexPose *>(_vertices[0]);
    const TebVertexPose *conf2 = static_cast<const TebVertexPose *>(_vertices[1]);
    const TebVertexTimeDiff *deltaT = static_cast<const TebVertexTimeDiff *>(_vertices[2]);
    const Eigen::Vector2d deltaS = conf2->estimate().GetPosition() - conf1->estimate().GetPosition();

    double cos_theta1 = std::cos(conf1->GetPose().GetTheta());
    double sin_theta1 = std::sin(conf1->GetPose().GetTheta());

    double r_dx = cos_theta1 * deltaS.x() + sin_theta1 * deltaS.y();
    double r_dy = -sin_theta1 * deltaS.x() + cos_theta1 * deltaS.y();

    double vx = r_dx / deltaT->estimate();
    double vy = r_dy / deltaT->estimate();
    double omega = g2o::normalize_theta(conf2->GetPose().GetTheta() - conf1->GetPose().GetTheta()) / deltaT->estimate();

    _error[0] = PenaltyBoundToInterval(vx, -config_param_->kinematics_opt().max_vel_x_backwards(),
                                       config_param_->kinematics_opt().max_vel_x(), config_param_->optimize_info().penalty_epsilon());
    _error[1] = PenaltyBoundToInterval(vy, config_param_->kinematics_opt().max_vel_y(), 0.0);
    _error[2] = PenaltyBoundToInterval(omega, config_param_->kinematics_opt().max_vel_theta(),
                                       config_param_->optimize_info().penalty_epsilon());

  }

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // namespace roborts_local_planner
#endif // ROBORTS_PLANNING_LOCAL_PLANNER_TEB_VELOCITY_EDGE_H
