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

#ifndef MODULES_PLANNING_LOCAL_PLANNER_UTILITY_TOOLS_H
#define MODULES_PLANNING_LOCAL_PLANNER_UTILITY_TOOLS_H

#include <math.h>
#include <stdlib.h>

#include <Eigen/Core>
#include <boost/utility.hpp>
#include <boost/type_traits.hpp>


namespace rrts {
namespace planning {
namespace local_planner {

#define SMALL_NUM 0.00000001

enum class RotType {
  LEFT,
  NONE,
  RIGHT
};

inline double LogisticSigmoid (double x) {
  return x / (1 + fabs(x));
}

inline double Cross2D (Eigen::Vector2d v1, Eigen::Vector2d v2) {
  return v1.x()*v2.y() - v2.x()*v1.y();
}

inline double Distance (double x0, double y0, double x1, double y1) {
  return hypot(x1 - x0, y1 - y0);
}

inline double PointToLineDistance (double px, double py, double x0, double y0, double x1, double y1) {
  double A_x = px - x0;
  double A_y = py - y0;
  double B_x = x1 - x0;
  double B_y = y1 - y0;

  double N_x = 1;
  double N_y = -(B_x / B_y);

  double N_module  = std::sqrt(std::pow(N_x, 2) + std::pow(N_y, 2));

  double U_x = N_x / N_module;
  double U_y = N_y / N_module;

  return abs(U_x * A_x + U_y * B_y);
}

inline double AverageAngles(const std::vector<double>& angles) {
  double x=0, y=0;
  for (std::vector<double>::const_iterator it = angles.begin(); it!=angles.end(); ++it) {
    x += cos(*it);
    y += sin(*it);
  }
  if(x == 0 && y == 0){
    return 0;
  } else {
    return std::atan2(y, x);
  }
}

} // namespace local_planner
} // namespace planning
} // namespace rrts

#endif // MODULES_PLANNING_LOCAL_PLANNER_UTILITY_TOOLS_H
