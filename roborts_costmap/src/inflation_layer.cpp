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
#include <algorithm>
#include <mutex>
#include "costmap_math.h"
#include "footprint.h"
#include "inflation_layer.h"
#include "inflation_layer_setting.pb.h"
namespace roborts_costmap {

InflationLayer::InflationLayer()
    : inflation_radius_(0),
      weight_(0),
      inflate_unknown_(false),
      cell_inflation_radius_(0),
      cached_cell_inflation_radius_(0),
      seen_(NULL),
      cached_costs_(NULL),
      cached_distances_(NULL),
      last_min_x_(-std::numeric_limits<float>::max()),
      last_min_y_(-std::numeric_limits<float>::max()),
      last_max_x_(std::numeric_limits<float>::max()),
      last_max_y_(std::numeric_limits<float>::max()) {
  inflation_access_ = new std::recursive_mutex();
}

void InflationLayer::OnInitialize() {

  std::unique_lock<std::recursive_mutex> lock(*inflation_access_);
  ros::NodeHandle nh("~/" + name_), g_nh;
  is_current_ = true;
  if (seen_)
    delete[] seen_;
  seen_ = NULL;
  seen_size_ = 0;
  need_reinflation_ = false;
  double inflation_radius, cost_scaling_factor;
  ParaInflationLayer para_inflation;
  roborts_common::ReadProtoFromTextFile(layered_costmap_->GetFilePath().c_str(), &para_inflation);
  inflation_radius = para_inflation.inflation_radius();
  cost_scaling_factor = para_inflation.cost_scaling_factor();
  need_reinflation_ = false;
  SetInflationParameters(inflation_radius, cost_scaling_factor);
  is_enabled_ = true;
  inflate_unknown_ = false;
  need_reinflation_ = true;
  MatchSize();
}

void InflationLayer::MatchSize() {
  std::unique_lock<std::recursive_mutex> lock(*inflation_access_);
  Costmap2D *costmap = layered_costmap_->GetCostMap();
  resolution_ = costmap->GetResolution();
  cell_inflation_radius_ = CellDistance(inflation_radius_);
  ComputeCaches();

  unsigned int size_x = costmap->GetSizeXCell(), size_y = costmap->GetSizeYCell();
  if (seen_)
    delete[] seen_;
  seen_size_ = size_x * size_y;
  seen_ = new bool[seen_size_];
}

void InflationLayer::UpdateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                  double *min_y, double *max_x, double *max_y) {
  if (need_reinflation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_reinflation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
  }
}

void InflationLayer::OnFootprintChanged() {
  inscribed_radius_ = layered_costmap_->GetInscribedRadius();
  cell_inflation_radius_ = CellDistance(inflation_radius_);
  ComputeCaches();
  need_reinflation_ = true;
}

void InflationLayer::UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
  std::unique_lock<std::recursive_mutex> lock(*inflation_access_);
  if (!is_enabled_ || (cell_inflation_radius_ == 0)) {
    ROS_ERROR("Layer is not enabled or inflation radius is zero");
    return;
  }
  unsigned char *master_array = master_grid.GetCharMap();
  unsigned int size_x = master_grid.GetSizeXCell(), size_y = master_grid.GetSizeYCell();

  if (seen_ == NULL) {
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  } else if (seen_size_ != size_x * size_y) {
    delete[] seen_;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  memset(seen_, false, size_x * size_y * sizeof(bool));

  min_i -= cell_inflation_radius_;
  min_j -= cell_inflation_radius_;
  max_i += cell_inflation_radius_;
  max_j += cell_inflation_radius_;

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(int(size_x), max_i);
  max_j = std::min(int(size_y), max_j);

  // Inflation list; we append cells to visit in a list associated with its distance to the nearest obstacle
  // We use a map<distance, list> to emulate the priority queue used before, with a notable performance boost

  // Start with lethal obstacles: by definition distance is 0.0
  std::vector<CellData> &obs_bin = inflation_cells_[0.0];
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.GetIndex(i, j);
      unsigned char cost = master_array[index];
      if (cost == LETHAL_OBSTACLE) {
        obs_bin.push_back(CellData(index, i, j, i, j));
      }
    }
  }

  // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  std::map<double, std::vector<CellData> >::iterator bin;
  for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin) {
    for (int i = 0; i < bin->second.size(); ++i) {
      // process all cells at distance dist_bin.first
      const CellData &cell = bin->second[i];

      unsigned int index = cell.index_;

      // ignore if already visited
      if (seen_[index]) {
        continue;
      }

      seen_[index] = true;

      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      unsigned int sx = cell.src_x_;
      unsigned int sy = cell.src_y_;

      // assign the cost associated with the distance from an obstacle to the cell
      unsigned char cost = CostLookup(mx, my, sx, sy);
      unsigned char old_cost = master_array[index];
      if (old_cost == NO_INFORMATION
          && (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
        master_array[index] = cost;
      else
        master_array[index] = std::max(old_cost, cost);

      // attempt to put the neighbors of the current cell onto the inflation list
      if (mx > 0)
        Enqueue(index - 1, mx - 1, my, sx, sy);
      if (my > 0)
        Enqueue(index - size_x, mx, my - 1, sx, sy);
      if (mx < size_x - 1)
        Enqueue(index + 1, mx + 1, my, sx, sy);
      if (my < size_y - 1)
        Enqueue(index + size_x, mx, my + 1, sx, sy);
    }
  }

  inflation_cells_.clear();
}

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at
 * @param  src_y The y index of the obstacle point inflation started at
 */
inline void InflationLayer::Enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                    unsigned int src_x, unsigned int src_y) {
  if (!seen_[index]) {
    // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
    double distance = DistanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
    if (distance > cell_inflation_radius_)
      return;

    // push the cell data onto the inflation list and mark
    inflation_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y));
  }
}

void InflationLayer::ComputeCaches() {
  if (cell_inflation_radius_ == 0)
    return;

  // based on the inflation radius... compute distance and cost caches
  if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
    DeleteKernels();
    //make a 2D array
    cached_costs_ = new unsigned char *[cell_inflation_radius_ + 2];
    cached_distances_ = new double *[cell_inflation_radius_ + 2];

    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i) {
      cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
      cached_distances_[i] = new double[cell_inflation_radius_ + 2];
      for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j) {
        cached_distances_[i][j] = hypot(i, j);
      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i) {
    for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j) {
      cached_costs_[i][j] = ComputeCost(cached_distances_[i][j]);
    }
  }
}

void InflationLayer::DeleteKernels() {
  if (cached_distances_ != NULL) {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i) {
      if (cached_distances_[i])
        delete[] cached_distances_[i];
    }
    if (cached_distances_)
      delete[] cached_distances_;
    cached_distances_ = NULL;
  }

  if (cached_costs_ != NULL) {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i) {
      if (cached_costs_[i])
        delete[] cached_costs_[i];
    }
    delete[] cached_costs_;
    cached_costs_ = NULL;
  }
}

void InflationLayer::SetInflationParameters(double inflation_radius, double cost_scaling_factor) {
  if (weight_ != cost_scaling_factor || inflation_radius_ != inflation_radius) {
    std::unique_lock<std::recursive_mutex> lock(*inflation_access_);
    inflation_radius_ = inflation_radius;
    cell_inflation_radius_ = CellDistance(inflation_radius_);
    weight_ = cost_scaling_factor;
    need_reinflation_ = true;
    ComputeCaches();
  }
}

}//namespace roborts_costmap
