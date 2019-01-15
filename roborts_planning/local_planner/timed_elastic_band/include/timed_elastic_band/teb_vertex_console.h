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

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_TEB_VERTEX_CONSOLE_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_TEB_VERTEX_CONSOLE_H

#include <complex>
#include <iterator>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>

#include "local_planner/obstacle.h"
#include "local_planner/data_base.h"
#include "local_planner/data_converter.h"
#include "local_planner/distance_calculation.h"

#include "timed_elastic_band/teb_vertex_pose.h"
#include "timed_elastic_band/teb_vertex_timediff.h"


namespace roborts_local_planner {

typedef std::vector<TebVertexPose *> PoseSequence;

typedef std::vector<TebVertexTimeDiff *> TimeDiffSequence;

/**
 * @brief graph vertices
 */
class TebVertexConsole {
 public:

  TebVertexConsole();

  virtual ~TebVertexConsole();


  PoseSequence &Poses() { return pose_vec_; };

  const PoseSequence &Poses() const {
    return pose_vec_;
  };

  TimeDiffSequence &TimeDiffs() {
    return timediff_vec_;
  };

  const TimeDiffSequence &TimeDiffs() const {
    return timediff_vec_;
  };

  double &TimeDiff(int index) {
    return timediff_vec_.at(index)->GetDiffTime();
  }

  const double &TimeDiff(int index) const {
    return timediff_vec_.at(index)->GetDiffTime();
  }

  DataBase &Pose(int index) {
    return pose_vec_.at(index)->GetPose();
  }

  const DataBase &Pose(int index) const {
    return pose_vec_.at(index)->GetPose();
  }

  DataBase &BackPose() {
    return pose_vec_.back()->GetPose();
  }

  const DataBase &BackPose() const {
    return pose_vec_.back()->GetPose();
  }

  double &BackTimeDiff() {
    return timediff_vec_.back()->GetDiffTime();
  }

  const double &BackTimeDiff() const {
    return timediff_vec_.back()->GetDiffTime();
  }

  TebVertexPose *PoseVertex(int index) {
    return pose_vec_.at(index);
  }

  TebVertexTimeDiff *TimeDiffVertex(int index) {
    return timediff_vec_.at(index);
  }

  void AddPose(const DataBase &pose, bool fixed = false);

  void AddPose(const Eigen::Ref<const Eigen::Vector2d> &position, double theta, bool fixed = false);

  void AddTimeDiff(double dt, bool fixed = false);

  void AddPoseAndTimeDiff(const DataBase &pose, double dt);

  void AddPoseAndTimeDiff(const Eigen::Ref<const Eigen::Vector2d> &position, double theta, double dt);

  void InsertPose(int index, const DataBase &pose);

  void InsertPose(int index, const Eigen::Ref<const Eigen::Vector2d> &position, double theta);

  void InsertTimeDiff(int index, double dt);

  void DeletePose(int index);

  void DeletePose(int index, int number);

  void DeleteTimeDiff(int index);

  void DeleteTimeDiff(int index, int number);

  bool InitTEBtoGoal(const DataBase &start,
                     const DataBase &goal,
                     double diststep = 0,
                     double timestep = 1,
                     int min_samples = 3,
                     bool guess_backwards_motion = false);

  template<typename BidirIter, typename Fun>
  bool InitTEBtoGoal(BidirIter path_start,
                     BidirIter path_end,
                     Fun fun_position,
                     double max_vel_x,
                     double max_vel_theta,
                     boost::optional<double> max_acc_x,
                     boost::optional<double> max_acc_theta,
                     boost::optional<double> start_orientation,
                     boost::optional<double> goal_orientation,
                     int min_samples = 3,
                     bool guess_backwards_motion = false);

  bool InitTEBtoGoal(std::vector<DataBase>& plan,
                     double dt,
                     bool estimate_orient = false,
                     int min_samples = 3,
                     bool guess_backwards_motion = false,
                     bool micro_control = false);

  void UpdateAndPruneTEB(boost::optional<const DataBase &> new_start,
                         boost::optional<const DataBase &> new_goal,
                         int min_samples = 3);

  void AutoResize(double dt_ref, double dt_hysteresis, int min_samples = 3, int max_samples = 1000);

  void SetPoseVertexFixed(int index, bool status);

  void SetTimeDiffVertexFixed(int index, bool status);

  void ClearAllVertex();

  int FindClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d> &ref_point,
                                double *distance = NULL,
                                int begin_idx = 0) const;

  int FindClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d> &ref_line_start,
                                const Eigen::Ref<const Eigen::Vector2d> &ref_line_end,
                                double *distance = NULL) const;

  int FindClosestTrajectoryPose(const Point2dContainer &vertices, double *distance = NULL) const;

  int FindClosestTrajectoryPose(const Obstacle &obstacle, double *distance = NULL) const;

  int SizePoses() const {
    return (int) pose_vec_.size();
  }

  int SizeTimeDiffs() const {
    return (int) timediff_vec_.size();
  }

  bool IsInit() const {
    return !timediff_vec_.empty() && !pose_vec_.empty();
  }

  double GetSumOfAllTimeDiffs() const;

  double GetAccumulatedDistance() const;

  bool DetectDetoursBackwards(double threshold = 0) const;

  bool IsTrajectoryInsideRegion(double radius, double max_dist_behind_robot = -1, int skip_poses = 0);

 protected:
  PoseSequence pose_vec_;
  TimeDiffSequence timediff_vec_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace roborts_local_planner
#include "timed_elastic_band.hpp"
#endif // ROBORTS_PLANNING_LOCAL_PLANNER_TEB_VERTEX_CONSOLE_H
