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

#ifndef ROBORTS_LOCALIZATION_TYPES_H
#define ROBORTS_LOCALIZATION_TYPES_H

#include <array>
#include <vector>
#include <functional>
#include <ostream>
#include <Eigen/Dense>

namespace roborts_localization {

// Use eigen3 as base data structure
using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;

using Mat3d = Eigen::Matrix3d;
using Mat2d = Eigen::Matrix2d;

using MatX2d = Eigen::MatrixX2d;

}// roborts_localization

#endif //ROBORTS_LOCALIZATION_TYPES_H
