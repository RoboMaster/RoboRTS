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

/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef MODULE_PERCEPTION_PLANNING_LOCALIZATION_AMCL_MATH_MATH_H
#define MODULE_PERCEPTION_PLANNING_LOCALIZATION_AMCL_MATH_MATH_H

#include <Eigen/Dense>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>

namespace rrts {
namespace perception {
namespace localization {
namespace math {

/**
 * @brief Column vector of 3 doubles
 */
using Vec3d = Eigen::Vector3d;

/**
 * @brief Column vector of 4 doubles
 */
using Vec4d = Eigen::Vector4d;

/**
 * @brief 2x2 Matrix of doubles
 */
using Mat2d = Eigen::Matrix2d;

/**
 * @brief 3x3 Matrix of doubles
 */
using Mat3d = Eigen::Matrix3d;

/**
 * @brief 6x6 Matrix of doubles
 */
using Mat6d = Eigen::Matrix<double,6,6>;

/**
 * @brief Eigenvalue decomposition: Symmetric matrix A => eigenvectors in columns of V,
 *                                  corresponding eigenvalues in d.
 * @param matrix_a Sysmmetric matrix A
 * @param matrix_v_ptr Eigenvectors in columns of matrix V
 * @param vector_d_ptr Eigenvalues in vector d
 */
void EigenDecomposition(const Mat3d &matrix_a,
						Mat3d *matrix_v_ptr,
						Vec3d *vector_d_ptr);

/**
 * @brief Angle normalization. Radian angle => [-pi,pi]
 * @param z Angle in radians
 * @return Returns normalized radian angle in [-pi,pi]
 */
double Normalize(double z);

/**
 * @brief Calculate difference between two angles.
 * @param a Angle a
 * @param b Angle b
 * @return Returns angle difference in radians.
 */
double AngleDiff(double a, double b);

/**
 * @brief Add coordinates of two pose vectors.
 * @param a Pose vector a (x, y , yaw)
 * @param b Pose vector a (x, y , yaw)
 * @return Returns the result pose.
 */
Vec3d CoordAdd(const Vec3d &a, const Vec3d &b);

}
}
}
}


#endif //MODULE_PERCEPTION_PLANNING_LOCALIZATION_AMCL_MATH_MATH_H
