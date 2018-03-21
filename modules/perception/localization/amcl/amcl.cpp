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

#include "modules/perception/localization/amcl/amcl.h"

namespace rrts {
namespace perception {
namespace localization {

std::vector<std::pair<int,int> > Amcl::free_space_indices;

void Amcl::Init() {

  CHECK(common::ReadProtoFromTextFile(proto_path, &amcl_param_))
  << "Error when loading amcl.prototxt";

  d_thresh_ = amcl_param_.update_min_d();
  a_thresh_ = amcl_param_.update_min_a();

  use_global_localization_ = amcl_param_.use_global_localization();

  UpdatePoseFromProto();

  cloud_pub_interval_.fromSec(1.0);

}


Amcl::~Amcl() {
  FreeMapDependentMemory();
  LOG_INFO << "Delete Amcl";
}

void Amcl::FreeMapDependentMemory() {
  if(map_ptr_!= nullptr){
    map_ptr_.reset();
  }
  if(pf_ptr_!= nullptr){
    pf_ptr_.reset();
  }
  laser_ptr_.reset();
}

void Amcl::HandleMapMessage(const nav_msgs::OccupancyGrid &map_msg) {

  FreeMapDependentMemory();
  map_ptr_.reset(new AmclMap());
  map_ptr_->ConvertFromMsg(map_msg);

  // Index of free space
  this->free_space_indices.resize(0);
  for(int i = 0; i < map_ptr_->GetSizeX(); i++){
    for(int j = 0; j < map_ptr_->GetSizeY(); j++){
      if(map_ptr_->CheckIndexFree(i,j)) {
        this->free_space_indices.push_back(std::make_pair(i, j));
      }
    }
  }

  pf_ptr_.reset(new ParticleFilter(amcl_param_.min_particles(),
                                   amcl_param_.max_particles(),
                                   amcl_param_.recovery_alpha_slow(),
                                   amcl_param_.recovery_alpha_fast(),
                                   (PfInitModelFunc)Amcl::UniformPoseGenerator,
                                   map_ptr_));

  pf_ptr_->SetKldParam(amcl_param_.kld_err(),amcl_param_.kld_z());
  UpdatePoseFromProto();

  math::Vec3d  pf_init_pose_mean;
  math::Mat3d  pf_init_pose_cov;
  pf_init_pose_cov.setZero();
  pf_init_pose_mean.setZero();
  pf_init_pose_mean = init_pose_;
  pf_init_pose_cov(0,0) = init_cov_(0);
  pf_init_pose_cov(1,1) = init_cov_(1);
  pf_init_pose_cov(2,2) = init_cov_(2);
  pf_ptr_->InitByGuassian(pf_init_pose_mean,pf_init_pose_cov);
  pf_init_ = false;

  odom_ptr_.reset(new SensorOdom);
  odom_ptr_->SetModelOmni(amcl_param_.odom_alpha1(),
                          amcl_param_.odom_alpha2(),
                          amcl_param_.odom_alpha3(),
                          amcl_param_.odom_alpha4(),
                          amcl_param_.odom_alpha5());

  laser_ptr_.reset(new SensorLaser(amcl_param_.laser_max_beams(),
                                   map_ptr_));
  //TODO(kevin.li) Other sensor model support.
  LOG_INFO << "Initializing likelihood field model; this can take some time on large maps";
  laser_ptr_->SetModelLikelihoodFieldProb
      (amcl_param_.z_hit(),
       amcl_param_.z_rand(),
       amcl_param_.sigma_hit(),
       amcl_param_.laser_likelihood_max_dist(),
       amcl_param_.do_beamskip(),
       amcl_param_.beam_skip_distance(),
       amcl_param_.beam_skip_threshold(),
       amcl_param_.beam_skip_error_threshold());
  LOG_INFO << "Done initializing likelihood field model.";

  if(use_global_localization_) {
    GlobalLocalization();
  } else{
    ApplyInitPose();
  }

}


void Amcl::UpdatePoseFromProto() {
  init_pose_[0] = 0.0;
  init_pose_[1] = 0.0;
  init_pose_[2] = 0.0;
  init_cov_[0] = 0.5 * 0.5;
  init_cov_[1] = 0.5 * 0.5;
  init_cov_[2] = (M_PI/12.0) * (M_PI/12.0);
  if(!std::isnan(amcl_param_.initial_pose_x())){
    init_pose_[0] = amcl_param_.initial_pose_x();
  } else{
    LOG_WARNING << "ignoring NAN in initial pose X position";
  }
  if(!std::isnan(amcl_param_.initial_pose_y())){
    init_pose_[1] = amcl_param_.initial_pose_y();
  } else
  {
    LOG_WARNING << "ignoring NAN in initial pose Y position";
  }
  if(!std::isnan(amcl_param_.initial_pose_a())){
    init_pose_[2] = amcl_param_.initial_pose_a();
  } else{
    LOG_WARNING << "ignoring NAN in initial pose Yaw";
  }
  if(!std::isnan(amcl_param_.initial_cov_xx())){
    init_cov_[0] = amcl_param_.initial_cov_xx();
  } else{
    LOG_WARNING << "ignoring NAN in initial covariance XX";
  }
  if(!std::isnan(amcl_param_.initial_cov_yy())){
    init_cov_[1] = amcl_param_.initial_cov_yy();
  } else{
    LOG_WARNING << "ignoring NAN in initial covariance YY";
  }
  if(!std::isnan(amcl_param_.initial_cov_aa())){
    init_cov_[1] = amcl_param_.initial_cov_aa();
  } else{
    LOG_WARNING << "ignoring NAN in initial covariance AA";
  }
}

void Amcl::HandleInitialPoseMessage(math::Vec3d pf_init_pose_mean,
                                    math::Mat3d pf_init_pose_cov) {
  initial_pose_hyp_.reset(new AmclHyp());
  initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
  initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
  ApplyInitPose();
}

void Amcl::ApplyInitPose() {
  if(initial_pose_hyp_ != nullptr && map_ptr_ != nullptr){
    pf_ptr_->InitByGuassian(initial_pose_hyp_->pf_pose_mean,
                            initial_pose_hyp_->pf_pose_cov);
    pf_init_= false;
    initial_pose_hyp_.reset();
  }
}

void Amcl::SetLaserSensorPose(math::Vec3d laser_pose) {
  laser_update_ = true;
  laser_ptr_->SetLaserPose(laser_pose);
}

math::Vec3d Amcl::UniformPoseGenerator(const std::shared_ptr<AmclMap> &map_ptr) {
  auto rand_index = static_cast<unsigned int>(drand48() * free_space_indices.size());
  std::pair<int,int> free_point = free_space_indices[rand_index];
  math::Vec3d p;
  map_ptr->ConvertMapCoordsToWorldCoords(free_point.first,free_point.second,p[0],p[1]);
  p[2] = drand48() * 2 * M_PI - M_PI;
  return p;
}


bool Amcl::GlobalLocalization(){
  if( map_ptr_ == nullptr ) {
    return true;
  }

  LOG_INFO << "Initializing with uniform distribution";
  PfInitModelFunc UniformPoseGeneratorFunc = Amcl::UniformPoseGenerator;
  pf_ptr_->InitByModel(UniformPoseGeneratorFunc,map_ptr_);

  LOG_INFO << "Global initialization done!";
  pf_init_ = false;
  return true;
}




int Amcl::Update(const math::Vec3d &pose,
                 const sensor_msgs::LaserScan &laser_scan,
                 const double &angle_min,
                 const double &angle_increment,
                 geometry_msgs::PoseArray &particle_cloud_pose_msg,
                 HypPose &hyp_pose){

  publish_pose_ = false;
  publish_particle_pose_cloud_ = false;
  update_tf_ = false;

  UpdateOdomPoseData(pose);

  resampled_ = false;
  if(laser_update_) {
    UpdateLaser(laser_scan, angle_min, angle_increment, pose, particle_cloud_pose_msg);
  }
  UpdateFilter(hyp_pose,laser_scan.header.stamp);


};

void Amcl::UpdateOdomPoseData(math::Vec3d pose)  {
  math::Vec3d delta;
  delta.setZero();

  if(pf_init_){

    // Compute change in pose
    delta[0] = pose[0] - pf_odom_pose_[0];
    delta[1] = pose[1] - pf_odom_pose_[1];
    delta[2] = math::AngleDiff(pose[2], pf_odom_pose_[2]);

    // See if we should update the filter
    bool update = std::fabs(delta[0]) > d_thresh_ ||
        std::fabs(delta[1]) > d_thresh_ ||
        std::fabs(delta[2]) > a_thresh_;

    // Set the laser update flags
    if(update){
      laser_update_ = true;
    }
  }

  force_publication_ = false;
  if(!pf_init_){
    // Pose at last filter update
    pf_odom_pose_ = pose;
    // Filter is now initialized"
    pf_init_ = true;
    // Set update sensor data flag
    laser_update_ = true;
    force_publication_ = true;
    resample_count_ = 0;
  }     // If the robot has moved, update the filter
  else if(pf_init_ && laser_update_){
    LOG_INFO << "Robot has moved, update the filter";
    SensorOdomData odom_data;
    odom_data.pose = pose;
    odom_data.delta = delta;
    odom_ptr_->UpdateAction(pf_ptr_,(SensorData*)&odom_data);
  }
}

void Amcl::UpdateLaser(const sensor_msgs::LaserScan &laser_scan,
                       double angle_min,
                       double angle_increment,
                       const math::Vec3d &pose,
                       geometry_msgs::PoseArray &particle_cloud_pose_msg) {

  SensorLaserData laser_data;
  laser_data.range_count = laser_scan.ranges.size();

  // Apply range min/max thresholds, if the user supplied them
  if(laser_max_range_ > 0.0) {
    laser_data.range_max = std::min(laser_scan.range_max,
                                    static_cast<float >(laser_max_range_));
  }
  else {
    laser_data.range_max = laser_scan.range_max;
  }

  double range_min;
  if(laser_min_range_ > 0.0) {
    range_min = std::max(laser_scan.range_min, static_cast<float >(laser_min_range_));
  }
  else{
    range_min = laser_scan.range_min;
  }

  laser_data.ranges_mat.resize(laser_data.range_count,2);
  laser_data.ranges_mat.setZero();

  for(int i=0;i<laser_data.range_count;i++)
  {
    // amcl doesn't (yet) have a concept of min range.  So we'll map short
    // readings to max range.
    if(laser_scan.ranges[i] <= range_min) {
      laser_data.ranges_mat(i, 0) = laser_data.range_max;
    }
    else {
      laser_data.ranges_mat(i, 0) = laser_scan.ranges[i];
    }
    // Compute bearing
    laser_data.ranges_mat(i,1) = angle_min + (i * angle_increment);
  }

  laser_ptr_->UpdateSensor(pf_ptr_,(SensorData*)(&laser_data));
  laser_update_ = false;

  pf_odom_pose_ = pose;

  particle_cloud_pose_msg = ResampleParticles();

}

geometry_msgs::PoseArray Amcl::ResampleParticles() {

  geometry_msgs::PoseArray particle_cloud_pose_msg;

  if (!(++resample_count_ % resample_interval_)) {
    LOG_INFO << "Resample the particles";
    pf_ptr_->UpdateResample();
    resampled_ = true;
  }

  auto set_ptr = pf_ptr_->GetCurrentSet();
  LOG_INFO << "Number of samples : " << set_ptr->sample_count;

  // Publish the resulting particle cloud
  particle_cloud_pose_msg.poses.resize(set_ptr->sample_count);
  for (int i = 0; i < set_ptr->sample_count; i++) {
    tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set_ptr->samples_vec[i].pose[2]),
                             tf::Vector3(set_ptr->samples_vec[i].pose[0],
                                         set_ptr->samples_vec[i].pose[1],
                                         0)),
                    particle_cloud_pose_msg.poses[i]);
  }
  publish_particle_pose_cloud_ = true;
  return particle_cloud_pose_msg;
}

void Amcl::UpdateFilter(HypPose &hyp_pose,
                        ros::Time laser_msg_timestamp) {

  if(resampled_ || force_publication_) {
    if (!resampled_) {
      LOG_INFO << "Recompute particle filter cluster statistics";
      pf_ptr_->ClusterStatistics();
    }
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<AmclHyp> hyps;
    hyps.resize(pf_ptr_->GetCurrentSet()->cluster_count);
    for (int hyp_count = 0;
         hyp_count < pf_ptr_->GetCurrentSet()->cluster_count;
         hyp_count++)
    {
      double weight;
      math::Vec3d pose_mean;
      math::Mat3d pose_cov;

      if (!pf_ptr_->GetClusterStatistics(hyp_count,
                                         &weight,
                                         &pose_mean,
                                         &pose_cov)) {
        LOG_ERROR << "Couldn't get stats on cluster " << hyp_count;
        break;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if (hyps[hyp_count].weight > max_weight) {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
    }

    if (max_weight > 0.0) {

      LOG_INFO << "Max weight pose: "
               << hyps[max_weight_hyp].pf_pose_mean[0]<< ", "
               << hyps[max_weight_hyp].pf_pose_mean[1]<< ", "
               << hyps[max_weight_hyp].pf_pose_mean[2];

      auto set = pf_ptr_->GetCurrentSet();

      hyp_pose.pose_mean = hyps[max_weight_hyp].pf_pose_mean;
      hyp_pose.pose_set_cov = set->covariant;

      publish_pose_ = true;
      update_tf_ = true;
    }
    else {
      LOG_ERROR << "No pose!";
      publish_pose_ = false;
      update_tf_ = false;
    }
  }
  else
  {
    // Nothing changed, republish the last transform.
    update_tf_ = false;
    publish_pose_ = true;

  }
}

bool Amcl::CheckPosePublish() {
  return publish_pose_;
}
bool Amcl::CheckParticlePoseCloudPublish() {
  return publish_particle_pose_cloud_;
}
bool Amcl::CheckTfUpdate() {
  return update_tf_;
}




}
}
}
