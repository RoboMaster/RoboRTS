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

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_TEB_KINEMATICS_EDGE_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_TEB_KINEMATICS_EDGE_H

#include <cmath>

#include "timed_elastic_band/teb_vertex_pose.h"
#include "timed_elastic_band/teb_penalties.h"
#include "timed_elastic_band/teb_base_eage.h"

#define USE_ANALYTIC_JACOBI

namespace roborts_local_planner {

class KinematicsDiffDriveEdge : public TebBinaryEdgeBase<2, double, TebVertexPose, TebVertexPose> {
 public:
  KinematicsDiffDriveEdge() {
    this->setMeasurement(0.);
  }

  void computeError() {
    const TebVertexPose *conf1 = static_cast<const TebVertexPose *>(_vertices[0]);
    const TebVertexPose *conf2 = static_cast<const TebVertexPose *>(_vertices[1]);

    Eigen::Vector2d deltaS = conf2->GetPose().GetPosition() - conf1->GetPose().GetPosition();

    _error[0] = fabs((cos(conf1->GetPose().GetTheta()) + cos(conf2->GetPose().GetTheta())) * deltaS[1]
                         - (sin(conf1->GetPose().GetTheta()) + sin(conf2->GetPose().GetTheta())) * deltaS[0]);

    Eigen::Vector2d angle_vec(cos(conf1->GetPose().GetTheta()), sin(conf1->GetPose().GetTheta()));
    _error[1] = PenaltyBoundFromBelow(deltaS.dot(angle_vec), 0, 0);

  }

#ifdef USE_ANALYTIC_JACOBI
#if 1

  void linearizeOplus() {
    const TebVertexPose *conf1 = static_cast<const TebVertexPose *>(_vertices[0]);
    const TebVertexPose *conf2 = static_cast<const TebVertexPose *>(_vertices[1]);

    Eigen::Vector2d deltaS = conf2->GetPose().GetPosition() - conf1->GetPose().GetPosition();

    double cos1 = cos(conf1->GetPose().GetTheta());
    double cos2 = cos(conf2->GetPose().GetTheta());
    double sin1 = sin(conf1->GetPose().GetTheta());
    double sin2 = sin(conf2->GetPose().GetTheta());
    double aux1 = sin1 + sin2;
    double aux2 = cos1 + cos2;

    double dd_error_1 = deltaS[0] * cos1;
    double dd_error_2 = deltaS[1] * sin1;
    double dd_dev = PenaltyBoundFromBelowDerivative(dd_error_1 + dd_error_2, 0, 0);

    double dev_nh_abs = g2o::sign((cos(conf1->GetPose().GetTheta()) + cos(conf2->GetPose().GetTheta())) * deltaS[1] -
        (sin(conf1->GetPose().GetTheta()) + sin(conf2->GetPose().GetTheta())) * deltaS[0]);

    _jacobianOplusXi(0, 0) = aux1 * dev_nh_abs;
    _jacobianOplusXi(0, 1) = -aux2 * dev_nh_abs;
    _jacobianOplusXi(1, 0) = -cos1 * dd_dev;
    _jacobianOplusXi(1, 1) = -sin1 * dd_dev;
    _jacobianOplusXi(0, 2) = (-dd_error_2 - dd_error_1) * dev_nh_abs;
    _jacobianOplusXi(1, 2) = (-sin1 * deltaS[0] + cos1 * deltaS[1]) * dd_dev;


    _jacobianOplusXj(0, 0) = -aux1 * dev_nh_abs;
    _jacobianOplusXj(0, 1) = aux2 * dev_nh_abs;
    _jacobianOplusXj(1, 0) = cos1 * dd_dev;
    _jacobianOplusXj(1, 1) = sin1 * dd_dev;
    _jacobianOplusXj(0, 2) = (-sin2 * deltaS[1] - cos2 * deltaS[0]) * dev_nh_abs;
    _jacobianOplusXj(1, 2) = 0;
  }
#endif
#endif

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class KinematicsCarlikeEdge : public TebBinaryEdgeBase<2, double, TebVertexPose, TebVertexPose> {
 public:

  KinematicsCarlikeEdge() {
    this->setMeasurement(0.);
  }

  void computeError() {
    const TebVertexPose *conf1 = static_cast<const TebVertexPose *>(_vertices[0]);
    const TebVertexPose *conf2 = static_cast<const TebVertexPose *>(_vertices[1]);

    Eigen::Vector2d deltaS = conf2->GetPose().GetPosition() - conf1->GetPose().GetPosition();

    _error[0] = fabs((cos(conf1->GetPose().GetTheta()) + cos(conf2->GetPose().GetTheta())) * deltaS[1]
                         - (sin(conf1->GetPose().GetTheta()) + sin(conf2->GetPose().GetTheta())) * deltaS[0]);

    double angle_diff = g2o::normalize_theta(conf2->GetPose().GetTheta() - conf1->GetPose().GetTheta());
    if (angle_diff == 0) {
      _error[1] = 0;
    } else if (config_param_->trajectory_opt().exact_arc_length()) {
      _error[1] = PenaltyBoundFromBelow(fabs(deltaS.norm() / (2 * sin(angle_diff / 2))),
                                        config_param_->kinematics_opt().min_turning_radius(), 0.0);
    } else {
      _error[1] = PenaltyBoundFromBelow(deltaS.norm() / fabs(angle_diff),
                                        config_param_->kinematics_opt().min_turning_radius(), 0.0);
    }

  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace roborts_local_planner
#endif // ROBORTS_PLANNING_LOCAL_PLANNER_TEB_KINEMATICS_EDGE_H
