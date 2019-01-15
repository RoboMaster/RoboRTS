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
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
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


#ifndef ROBORTS_LOCALIZATION_AMCL_AMCL_H
#define ROBORTS_LOCALIZATION_AMCL_AMCL_H

#include <mutex>
#include <functional>

#include <tf/tf.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include "log.h"
#include "localization_math.h"
#include "types.h"

#include "map/amcl_map.h"
#include "particle_filter/particle_filter.h"
#include "sensors/sensor_laser.h"
#include "sensors/sensor_odom.h"
#include "amcl_config.h"

namespace roborts_localization {
/**
 * @brief Pose hypothesis type in localization module
 */
class HypPose {
 public:
  //! Mean of pose esimate
  Vec3d pose_mean;
  //! Covariance of pose estimate
  Mat3d pose_set_cov;
};

/**
 * @brief Pose hypothesis type in Amcl
 */
class AmclHyp {
 public:
  //! Total weight (weights sum to 1)
  double weight;
  //! Mean of pose esimate
  Vec3d pf_pose_mean;
  //! Covariance of pose estimate
  Mat3d pf_pose_cov;
};

/**
 * @brief Amcl algorithm wrapping from ROS-Navigation
 */
class Amcl {
 public:

  /**
   * @brief Amcl initialization function
   */
  void Init(const Vec3d &init_pose, const Vec3d &init_cov);

  /**
   * @brief Amcl destructor.
   */
  ~Amcl();

  void GetParamFromRos(ros::NodeHandle *nh);

  /**
   * @brief Map message handler
   * @param map_msg Static Map message
   */
  void HandleMapMessage(const nav_msgs::OccupancyGrid &map_msg, const Vec3d &init_pose, const Vec3d &init_cov);

  /**
   * @brief Initial pose estimation message handler
   * @param pf_init_pose_mean Initial mean of pose estimation
   * @param pf_init_pose_cov Initial covariance of pose estimation
   */
  void HandleInitialPoseMessage(Vec3d pf_init_pose_mean,
                                Mat3d pf_init_pose_cov);

  /**
   * @brief Set up laser pose in base frame
   * @param laser_pose Laser pose in base frame
   */
  void SetLaserSensorPose(Vec3d laser_pose);

  /**
   * @brief Update AMCL algorithm
   * @param pose Odom pose
   * @param laser_scan Laser_scan msg
   * @param angle_min Laser scan msg angle min in base frame
   * @param angle_increment Laser scan msg angle increment in base frame
   * @param particle_cloud_pose_msg Particle cloud pose msg to publish
   * @param hyp_pose Pose hypothesis to publish
   * @return Error code
   */
  int Update(const Vec3d &pose,
             const sensor_msgs::LaserScan &laser_scan,
             const double &angle_min,
             const double &angle_increment,
             geometry_msgs::PoseArray &particle_cloud_pose_msg,
             HypPose &hyp_pose
  );

  void UpdateUwb(const Vec3d &uwb_pose, const Vec3d &uwb_cov);

  void UpdateUwb(const Vec3d &uwb_pose);

  /**
   * @brief Check amcl pose publish state
   * @return Return true if publish amcl pose
   */
  bool CheckPosePublish();

  /**
   * @brief Check particle cloud pose publish state
   * @return Return true if publish particle cloud pose
   */
  bool CheckParticlePoseCloudPublish();

  /**
   * @brief Check TF update state
   * @return Return true if need update TF
   */
  bool CheckTfUpdate();

  bool RandomHeadingGlobalLocalization();

  const nav_msgs::OccupancyGrid &GetDistanceMapMsg();

 private:
  void Reset();

  bool GlobalLocalization();

  void UpdatePoseFromParam(const Vec3d &init_pose, const Vec3d &init_cov);
  void ApplyInitPose();

  void UpdateOdomPoseData(Vec3d pose);

  void UpdateLaser(const sensor_msgs::LaserScan &laser_scan,
                   double angle_min,
                   double angle_increment,
                   const Vec3d &pose,
                   geometry_msgs::PoseArray &particle_cloud_pose_msg);

  void UpdateFilter(HypPose &hyp_pose,
                    ros::Time laser_msg_timestamp);

  geometry_msgs::PoseArray ResampleParticles();

  Vec3d UniformPoseGenerator();

 public:
  AmclConfig amcl_param_;

 private:
  std::mutex mutex_;

  int max_uwb_particles_ = 1;
  double uwb_cov_x_ = 0.01;
  double uwb_cov_y_ = 0.01;
  double resample_uwb_factor_ = 3.0;

  double d_thresh_ = 0.2;
  double a_thresh_ = M_PI / 6.0;
  int resample_interval_ = 2;
  int resample_count_ = 0;
  double laser_min_range_ = 0;
  double laser_max_range_ = 0;
  double total_weight_ = 0;

  Vec3d init_pose_;
  Vec3d init_cov_;

  std::unique_ptr<AmclHyp> initial_pose_hyp_ = nullptr;

  Vec3d pf_odom_pose_;

  std::shared_ptr<AmclMap> map_ptr_;

  ParticleFilterPtr pf_ptr_;
  bool pf_init_ = true;

  std::unique_ptr<SensorLaser> laser_model_ptr_;
  std::unique_ptr<SensorOdom> odom_model_ptr_;

  bool use_global_localization_ = true;
  bool random_heading_ = true;

  ros::Duration cloud_pub_interval_;

  static std::vector<std::pair<int, int> > free_space_indices;

  bool laser_update_ = true;
  bool resampled_ = false;
  bool force_publication_ = false;

  //Publish flag
  bool publish_particle_pose_cloud_ = false;
  bool publish_pose_ = false;
  bool update_tf_ = false;

};

}// roborts_localization

#endif //ROBORTS_LOCALIZATION_AMCL_AMCL_H
