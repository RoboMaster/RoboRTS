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

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_TEB_PENALTIES_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_TEB_PENALTIES_H

#include <cmath>
#include <Eigen/Core>
#include <g2o/stuff/misc.h>

namespace roborts_local_planner {

inline double PenaltyBoundToInterval(const double &var, const double &a, const double &epsilon) {
  if (var < -a + epsilon) {
    return (-var - (a - epsilon));
  }
  if (var <= a - epsilon) {
    return 0.;
  } else {
    return (var - (a - epsilon));
  }
}

inline double PenaltyBoundToInterval(const double &var, const double &a, const double &b, const double &epsilon) {
  if (var < a + epsilon) {
    return (-var + (a + epsilon));
  }
  if (var <= b - epsilon) {
    return 0.;
  } else {
    return (var - (b - epsilon));
  }
}

inline double PenaltyBoundFromBelow(const double &var, const double &a, const double &epsilon) {
  if (var >= a + epsilon) {
    return 0.;
  } else {
    return (-var + (a + epsilon));
  }
}

inline double PenaltyBoundToIntervalDerivative(const double &var, const double &a, const double &epsilon) {
  if (var < -a + epsilon) {
    return -1;
  }
  if (var <= a - epsilon) {
    return 0.;
  } else {
    return 1;
  }
}

inline double PenaltyBoundToIntervalDerivative(const double &var,
                                               const double &a,
                                               const double &b,
                                               const double &epsilon) {
  if (var < a + epsilon) {
    return -1;
  }
  if (var <= b - epsilon) {
    return 0.;
  } else {
    return 1;
  }
}

inline double PenaltyBoundFromBelowDerivative(const double &var, const double &a, const double &epsilon) {
  if (var >= a + epsilon) {
    return 0.;
  } else {
    return -1;
  }
}

} // namespace roborts_local_planner
#endif // ROBORTS_PLANNING_LOCAL_PLANNER_TEB_PENALTIES_H
