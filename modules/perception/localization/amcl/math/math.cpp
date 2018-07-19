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

#include "modules/perception/localization/amcl/math/math.h"

namespace rrts {
namespace perception {
namespace localization {

void math::EigenDecomposition(const math::Mat3d &matrix_a,
							  math::Mat3d *matrix_v_ptr,
							  math::Vec3d *vector_d_ptr) {

	Eigen::EigenSolver<math::Mat3d> eigen_solver(matrix_a);
	eigen_solver.compute(matrix_a);
	if (eigen_solver.info() == Eigen::Success) {
		*matrix_v_ptr = eigen_solver.eigenvectors().real();
		*vector_d_ptr = eigen_solver.eigenvalues().real();

		math::Vec3d::Index idx[3];
		vector_d_ptr->maxCoeff(&idx[2]);
		vector_d_ptr->minCoeff(&idx[0]);
		idx[1] = 0;
		while (idx[1] < 3) {
			if (idx[1] != idx[0] && idx[1] != idx[2]) {
				break;
			} else {
				idx[1]++;
			}
		}

		math::Vec3d tmp_vec;
		math::Mat3d tmp_mat;

		for (int i = 0; i < 3; i++) {
			tmp_vec.row(i) = vector_d_ptr->row(idx[i]);
			tmp_mat.col(i) = matrix_v_ptr->col(idx[i]);
		}

		*matrix_v_ptr << tmp_mat;
		*vector_d_ptr << tmp_vec;

	}
}

math::Vec3d math::CoordAdd(const math::Vec3d &a, const math::Vec3d &b) {
	math::Vec3d c;
	c(0) = b(0) + a(0) * std::cos(b(2)) - a(1) * std::sin(b(2));
	c(1) = b(1) + a(0) * std::sin(b(2)) - a(1) * std::cos(b(2));
	c(2) = b(2) + a(2);
	c(2) = std::atan2(sin(c(2)), cos(c(2)));
	return c;
}

}
}
}
