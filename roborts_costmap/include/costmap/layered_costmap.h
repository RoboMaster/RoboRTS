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
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *********************************************************************/
#ifndef ROBORTS_COSTMAP_COSTMAPLAYERS_H
#define ROBORTS_COSTMAP_COSTMAPLAYERS_H

#include <geometry_msgs/Point.h>
#include "map_common.h"
#include "layer.h"
#include "costmap_2d.h"
#include "footprint.h"

namespace roborts_costmap {

class Layer;

class CostmapLayers {
 public:
  CostmapLayers(std::string global_frame, bool rolling_window, bool track_unknown);
  ~CostmapLayers();
  void UpdateMap(double robot_x, double robot_y, double robot_yaw);

  const std::vector<geometry_msgs::Point>& GetFootprint() {
    return footprint_;
  }

  std::string GetGlobalFrameID() const {
    return global_frame_id_;
  }

  bool IsRollingWindow() const {
    return is_rolling_window_;
  }

  bool IsSizeLocked() const {
    return is_size_locked_;
  }

  bool IsTrackingUnknown() const {
    return costmap_.GetDefaultValue() == NO_INFORMATION;
  }

  void ResizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y,
                 bool size_locked = false);

  Costmap2D* GetCostMap() {
    return &costmap_;
  }

  void GetUpdatedBounds(double& minx, double& miny, double& maxx, double& maxy) {
    minx = minx_;
    miny = miny_;
    maxx = maxx_;
    maxy = maxy_;
  }

  bool IsCurrent();

  bool IsRolling() {
    return is_rolling_window_;
  }

  std::vector<Layer*>* GetPlugins() {
    return &plugins_;
  }

  void AddPlugin(Layer* plugin) {
    plugins_.push_back(plugin);
  }

  bool IsSizeLocked()
  {
    return is_size_locked_;
  }

  void GetBounds(unsigned int* x0, unsigned int* xn, unsigned int* y0, unsigned int* yn)
  {
    *x0 = bx0_;
    *xn = bxn_;
    *y0 = by0_;
    *yn = byn_;
  }

  bool IsInitialized()
  {
    return is_initialized_;
  }

  /** @brief Updates the stored footprint, updates the circumscribed
   * and inscribed radii, and calls onFootprintChanged() in all
   * layers. */
  void SetFootprint(const std::vector<geometry_msgs::Point>& footprint_spec);

  /** @brief The radius of a circle centered at the origin of the
   * robot which just surrounds all points on the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  double GetCircumscribedRadius() { return circumscribed_radius_; }

  /** @brief The radius of a circle centered at the origin of the
   * robot which is just within all points and edges of the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  double GetInscribedRadius() { return inscribed_radius_; }
  /**
   * @brief Set the inflation layer prototxt path for the inflaiton layer to read in
   * @param path The file path.
   */
  void SetFilePath(const std::string &path) {
    file_path_ = path;
  }

  std::string GetFilePath() const {
    return file_path_;
  }

 private:
  std::string global_frame_id_, file_path_;
  std::vector<geometry_msgs::Point> footprint_;
  Costmap2D costmap_;
  bool is_rolling_window_, is_size_locked_, is_initialized_, is_current_;
  double  minx_, miny_, maxx_, maxy_, circumscribed_radius_, inscribed_radius_;
  unsigned int bx0_, bxn_, by0_, byn_;
  std::vector<Layer*> plugins_;
};

} //namespace roborts_costmap
#endif //ROBORTS_COSTMAP_COSTMAPLAYERS_H
