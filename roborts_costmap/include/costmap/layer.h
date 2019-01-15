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
#ifndef ROBORTS_COSTMAP_LAYER_H
#define ROBORTS_COSTMAP_LAYER_H

#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "costmap_2d.h"
#include "layered_costmap.h"

namespace roborts_costmap {

class CostmapLayers;

class Layer {
 public:
/**
 * @brief constructor
 */
  Layer();
/**
 * @brief initialize
 * @param parent the layered costmap, ie master grid
 * @param name this layer name
 * @param tf a tf listener providing transforms
 */
  void Initialize(CostmapLayers *parent, std::string name, tf::TransformListener *tf);

/**
 * @brief This is called by the LayeredCostmap to poll this plugin as to how
 *        much of the costmap it needs to update. Each layer can increase
 *        the size of this bounds. *
 * @param robot_x
 * @param robot_y
 * @param robot_yaw these point the pose of the robot in global frame
 * @param min_x
 * @param min_y
 * @param max_x
 * @param max_y these declare the updating boundary
 */
  virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                            double *max_x, double *max_y) {}

/**
 * @brief Actually update the underlying costmap, only within the bounds
 *        calculated during UpdateBounds(). *
 * @param master_grid the master map
 * @param min_i
 * @param min_j
 * @param max_i
 * @param max_j the update boundary
 */
  virtual void UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {}

  /**
   * @brief Stop.
   */
  virtual void Deactivate() {}

  /**
   * @brief Restart, if they've been stopped.
   */
  virtual void Activate() {}

  /**
   * @brief Reset the layer
   */
  virtual void Reset() {}

  virtual ~Layer() {}

  /**
   * @brief Check to make sure all the data in the layer is update.
   * @return Whether the data in the layer is up to date.
   */
  bool IsCurrent() const {
    return is_current_;
  }

  /**
   * @brief Implement this to make this layer match the size of the parent costmap.
   */
  virtual void MatchSize() {}

  std::string GetName() const {
    return name_;
  }

  /**
   * @brief Convenience function for layered_costmap_->GetFootprint().
   */
  const std::vector<geometry_msgs::Point> &GetFootprint() const;

  virtual void OnFootprintChanged() {}

 protected:
  /** @brief This is called at the end of initialize().  Override to
   * implement subclass-specific initialization.
   * tf_, name_, and layered_costmap_ will all be set already when this is called. */
  virtual void OnInitialize() {}

  CostmapLayers *layered_costmap_;
  bool is_current_, is_enabled_, is_debug_;
  std::string name_;
  tf::TransformListener *tf_;

 private:
  std::vector<geometry_msgs::Point> footprint_spec_;
};

}  //namespace roborts_costmap
#endif  // ROBORTS_COSTMAP_LAYER_H
