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


#ifndef MODULE_PERCEPTION_PLANNING_LOCALIZATION_AMCL_PARTICLE_FILTER_PARTICLE_FILTER_H
#define MODULE_PERCEPTION_PLANNING_LOCALIZATION_AMCL_PARTICLE_FILTER_PARTICLE_FILTER_H

#include <vector>
#include <random>
#include <memory>

#include "modules/perception/localization/amcl/math/math.h"
#include "modules/perception/localization/amcl/map/amcl_map.h"
#include "modules/perception/localization/amcl/particle_filter/particle_filter_sample.h"



namespace rrts {
namespace perception {
namespace localization {

class ParticleFilter;

using SensorModelFunc = double (*)(void *sensor_data, ParticleFilterSampleSet *set);
using SensorActionModelFunc = void (*)(void *action_data, ParticleFilterSampleSet *set);

using ParticleFilterPtr = std::shared_ptr<ParticleFilter>;
using SampleSetPtr = std::shared_ptr<ParticleFilterSampleSet>;

using PfInitModelFunc = std::function<math::Vec3d(std::shared_ptr<AmclMap> map_ptr)> ;

/**
 * @brief Particle filter class
 */
class ParticleFilter {
  friend class SensorLaser;
  friend class SensorOdom;
 public:
  /**
   * @brief Constructor of particle filter class
   * @param min_samples Min number of samples
   * @param max_samples Max number of samples
   * @param alpha_slow Decay rates for running averages
   * @param alpha_fast Decay rates for running averages
   * @param random_pose_func Function used to draw random pose samples
   * @param random_pose_data
   */
  ParticleFilter(int min_samples,
                 int max_samples,
                 double alpha_slow,
                 double alpha_fast,
                 const PfInitModelFunc &random_pose_func,
                 const std::shared_ptr<AmclMap> &map_ptr
  );

  /**
   * @brief Destructor
   */
  virtual ~ParticleFilter();

  /**
   * @brief Initialize the filter using a guassian pdf
   * @param mean Initial pose mean
   * @param cov Initial pose covariant
   */
  void InitByGuassian(const math::Vec3d &mean, const math::Mat3d &cov);

  /**
   * @brief Initialize the filter using some model
   * @param init_fn Model function
   * @param init_data Model data
   */
  void InitByModel(PfInitModelFunc init_fn, const std::shared_ptr<AmclMap> &init_data);

  /**
   * @brief Resample the distribution
   */
  void UpdateResample();

  /**
   * @brief Compute the required number of samples,
   *        given that there are k bins with samples in them.
   *        This is taken directly from Fox et al.
   * @param k Bins with samples
   * @return Returns the required number of samples for resampling.
   */
  int ResampleLimit(int k);

  /**
   * @brief Re-compute the cluster statistics for a sample set
   */
  void ClusterStatistics();

  /**
   * @brief Compute the statistics for a particular cluster.
   * @param clabel Cluster label
   * @param weight Weight of the cluster
   * @param mean Mean of the cluster
   * @param cov Cov of the cluster
   * @return Returns 0 if there is no such cluster.
   */
  int GetClusterStatistics(int clabel, double *weight,
                           math::Vec3d *mean, math::Mat3d *cov);

  void SetKldParam(double pop_err,double pop_z);

  SampleSetPtr GetCurrentSet() const;

 private:
  void ClusterStatistics(const SampleSetPtr &sample_set_ptr);
  void InitConverged();
  bool UpdateConverged();
 private:

  //! This min and max number of samples
  int min_samples_ = 0, max_samples_ = 0;

  //! Population size parameters
  double pop_err_, pop_z_;

  //! The sample sets.  We keep two sets and use [current_set] to identify the active set.
  int current_set_;

  std::array<SampleSetPtr, 2> sample_set_ptr_array_;

  //! Running averages, slow and fast, of likelihood
  double w_slow_, w_fast_;

  //! Decay rates for running averages
  double alpha_slow_, alpha_fast_;

  //! Function used to draw random pose samples
  PfInitModelFunc random_pose_func_;
  std::shared_ptr<AmclMap> map_ptr_;

  //! Distance threshold in each axis over which the pf is considered to not be converged
  double dist_threshold_ = 0.5;
  //! Particle filter converged flag
  bool converged_ = false;
};

}
}
}


#endif //MODULE_PERCEPTION_PLANNING_LOCALIZATION_AMCL_PARTICLE_FILTER_PARTICLE_FILTER_H
