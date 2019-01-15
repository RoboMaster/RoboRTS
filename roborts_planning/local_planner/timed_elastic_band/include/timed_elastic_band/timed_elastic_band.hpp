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

#include "timed_elastic_band/teb_vertex_console.h"

namespace roborts_local_planner {

template<typename BidirIter, typename Fun>
bool TebVertexConsole::InitTEBtoGoal(BidirIter path_start,
                                     BidirIter path_end,
                                     Fun fun_position,
                                     double max_vel_x,
                                     double max_vel_theta,
                                     boost::optional<double> max_acc_x,
                                     boost::optional<double> max_acc_theta,
                                     boost::optional<double> start_orientation,
                                     boost::optional<double> goal_orientation,
                                     int min_samples,
                                     bool guess_backwards_motion) {
  Eigen::Vector2d start_position = fun_position(*path_start);
  Eigen::Vector2d goal_position = fun_position(*boost::prior(path_end));

  bool backwards = false;

  double start_orient, goal_orient;
  if (start_orientation) {
    start_orient = *start_orientation;

    if (guess_backwards_motion
        && (goal_position - start_position).dot(Eigen::Vector2d(std::cos(start_orient), std::sin(start_orient))) < 0)
      backwards = true;
  } else {
    Eigen::Vector2d start2goal = goal_position - start_position;
    start_orient = atan2(start2goal[1], start2goal[0]);
  }
  double timestep = 1;


  if (goal_orientation) {
    goal_orient = *goal_orientation;
  } else {
    goal_orient = start_orient;
  }

  if (!IsInit()) {
    AddPose(start_position, start_orient, true);

    std::advance(path_start, 1);
    std::advance(path_end, -1);
    int idx = 0;
    for (; path_start != path_end; ++path_start)
    {
      Eigen::Vector2d curr_point = fun_position(*path_start);
      Eigen::Vector2d
          diff_last = curr_point - Pose(idx).GetPosition();
      double diff_norm = diff_last.norm();

      double timestep_vel = diff_norm / max_vel_x;
      double timestep_acc;
      if (max_acc_x) {
        timestep_acc = sqrt(2 * diff_norm / (*max_acc_x));
        if (timestep_vel < timestep_acc && max_acc_x) timestep = timestep_acc;
        else timestep = timestep_vel;
      } else timestep = timestep_vel;

      if (timestep < 0) timestep = 0.2;

      double yaw = atan2(diff_last[1], diff_last[0]);
      if (backwards)
        yaw = g2o::normalize_theta(yaw + M_PI);
      AddPoseAndTimeDiff(curr_point, yaw, timestep);

      Eigen::Vector2d diff_next = fun_position(*boost::next(path_start))
          - curr_point;
      double ang_diff = std::abs(g2o::normalize_theta(atan2(diff_next[1], diff_next[0])
                                                          - atan2(diff_last[1], diff_last[0])));

      timestep_vel = ang_diff / max_vel_theta;
      if (max_acc_theta) {
        timestep_acc = sqrt(2 * ang_diff / (*max_acc_theta));
        if (timestep_vel < timestep_acc) timestep = timestep_acc;
        else timestep = timestep_vel;
      } else timestep = timestep_vel;

      if (timestep < 0) timestep = 0.2;

      yaw = atan2(diff_last[1], diff_last[0]);
      if (backwards)
        yaw = g2o::normalize_theta(yaw + M_PI);
      AddPoseAndTimeDiff(curr_point, yaw, timestep);

      ++idx;
    }
    Eigen::Vector2d diff = goal_position - Pose(idx).GetPosition();
    double diff_norm = diff.norm();
    double timestep_vel = diff_norm / max_vel_x;
    if (max_acc_x) {
      double timestep_acc = sqrt(2 * diff_norm / (*max_acc_x));
      if (timestep_vel < timestep_acc) timestep = timestep_acc;
      else timestep = timestep_vel;
    } else timestep = timestep_vel;

    DataBase goal(goal_position, goal_orient);


    if (SizePoses() < min_samples - 1) {
      while (SizePoses() < min_samples - 1) {

        DataBase temp_data;
        temp_data.AverageInPlace(BackPose(), goal);

        AddPoseAndTimeDiff(temp_data, timestep);
      }
    }

    AddPoseAndTimeDiff(goal, timestep);
    SetPoseVertexFixed(SizePoses() - 1, true);
  } else {
    return false;
  }
  return true;
}

} // namespace roborts_local_planner

