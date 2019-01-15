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

#ifndef ROBORTS_LOCALIZATION_AMCL_PARTICLE_FILTER_PARTICLE_FILTER_SAMPLE_H
#define ROBORTS_LOCALIZATION_AMCL_PARTICLE_FILTER_PARTICLE_FILTER_SAMPLE_H

#include "localization_math.h"
#include "particle_filter_kdtree.h"

namespace roborts_localization {

/**
 * @brief Information for a single sample.
 */
class ParticleFilterSample {
 public:
  ParticleFilterSample() {
    Reset();
  }
  /**
   * @brief Particle filter sample initialize
   */
  void Reset() {
    pose.setZero();
    weight = 0;
  };
 public:
  //! Pose represented by this sample
  Vec3d pose;

  //! Weight for this pose
  double weight;
};

/**
 * @brief Information for a cluster of samples
 */
class ParticleFilterCluster {
 public:
  ParticleFilterCluster() {
    Reset();
  }
  void Reset() {
    count = 0;
    weight = 0;
    mean.setZero();
    cov.setZero();
    ws_vec.setZero();
    ws_mat.setZero();
  }
 public:
  //! Number of samples
  int count;
  //! Total weight of samples in this cluster
  double weight;
  //! Cluster statistics mean
  Vec3d mean;
  //! Cluster statistics cov
  Mat3d cov;
  //! Workspace 4 double vector
  Vec4d ws_vec;
  //! Workspace 2x2 double matrix
  Mat2d ws_mat;
};

/**
 * @brief Information for a set of samples
 */
class ParticleFilterSampleSet {
 public:
  //! Number of samples
  int sample_count = 0;
  //! Vector of samples
  std::vector<ParticleFilterSample> samples_vec;
  //! Number of Clusters
  int cluster_count = 0;
  //! Max number of clusters
  int cluster_max_count = 0;
  //! Vector of clusters
  std::vector<ParticleFilterCluster> clusters_vec;
  //! A kdtree encoding the histogram
  std::unique_ptr<ParticleFilterKDTree> kd_tree_ptr;
  //! Filter statistics mean
  Vec3d mean;
  //! Filter statistics covariant
  Mat3d covariant;
  //! Filter statistics converged status
  bool converged = false;
};

}// roborts_localization

#endif //ROBORTS_LOCALIZATION_AMCL_PARTICLE_FILTER_PARTICLE_FILTER_SAMPLE_H
