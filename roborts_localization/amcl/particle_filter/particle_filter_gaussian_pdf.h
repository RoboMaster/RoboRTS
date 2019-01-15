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

#ifndef ROBORTS_LOCALIZATION_AMCL_PARTICLE_FILTER_PARTICLE_FILTER_GAUSSIAN_PDF_H
#define ROBORTS_LOCALIZATION_AMCL_PARTICLE_FILTER_PARTICLE_FILTER_GAUSSIAN_PDF_H

#include <random>

#include "localization_math.h"
#include "log.h"

namespace roborts_localization {

/**
 * @brief Gaussian pdf class
 */
class ParticleFilterGaussianPdf {

 public:
  /**
   * @brief Default constructor
   */
  ParticleFilterGaussianPdf() = default;
  /**
   * @brief Create a gaussian pdf by mean and covariance
   * @param mean Mean to initialize the gaussian pdf
   * @param covariance Covariance initialize the gaussian pdf
   */
  ParticleFilterGaussianPdf(const Vec3d &mean, const Mat3d &covariance);

  /**
   * @brief Generate random pose particle sample
   * @return Return the random pose particle sample
   */
  Vec3d GenerateSample();

 private:

  Vec3d mean_;
  Mat3d covariance_;

  // Decomposed covariance matrix (rotation * diagonal)
  Mat3d covariance_rotation_;
  Vec3d covariance_diagonal_;

};

}// roborts_localization

#endif //ROBORTS_LOCALIZATION_AMCL_PARTICLE_FILTER_PARTICLE_FILTER_GAUSSIAN_PDF_H
