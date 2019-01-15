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
 *  This library is distrmathibuted in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "particle_filter_gaussian_pdf.h"

namespace roborts_localization {

ParticleFilterGaussianPdf::ParticleFilterGaussianPdf(const Vec3d &mean,
                                                     const Mat3d &covariance) {
  this->mean_ = mean;
  this->covariance_ = covariance;
  math::EigenDecomposition(this->covariance_, this->covariance_rotation_, this->covariance_diagonal_);
}

Vec3d ParticleFilterGaussianPdf::GenerateSample() {
  int i = 0;
  Vec3d random_vec;
  Vec3d mean_vec;

  for (i = 0; i < this->covariance_diagonal_.size(); i++) {
    double sigma = this->covariance_diagonal_(i);
    random_vec(i) = math::RandomGaussianNumByStdDev<double>(sigma);
  }

  for (i = 0; i < this->mean_.size(); i++) {
    mean_vec(i) = this->mean_(i);
    for (int j = 0; j < this->mean_.size(); ++j) {
      mean_vec(i) += this->covariance_rotation_(i, j) * random_vec(j);
    }
  }

  return mean_vec;
}

}// roborts_localization


