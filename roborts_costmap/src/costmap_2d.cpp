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

#include "costmap_2d.h"

namespace roborts_costmap {

Costmap2D::Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution, double origin_x, \
                     double origin_y, unsigned char default_value) : size_x_(cells_size_x),
                                                                     size_y_(cells_size_y), \

                                                                     resolution_(resolution),
                                                                     origin_x_(origin_x),
                                                                     origin_y_(origin_y),
                                                                     costmap_(NULL),
                                                                     default_value_(default_value) {
  access_ = new mutex_t();
  InitMaps(size_x_, size_y_);
  ResetMaps();
}

void Costmap2D::DeleteMaps() {
  std::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  costmap_ = NULL;
}

void Costmap2D::InitMaps(unsigned int size_x, unsigned int size_y) {
  std::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  costmap_ = new unsigned char[size_x * size_y];
}

void Costmap2D::ResizeMap(unsigned int size_x,
                          unsigned int size_y,
                          double resolution,
                          double origin_x,
                          double origin_y) {
  size_x_ = size_x;
  size_y_ = size_y;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  InitMaps(size_x, size_y);
  ResetMaps();
}

void Costmap2D::ResetMaps() {
  std::unique_lock<mutex_t> lock(*access_);
  memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

void Costmap2D::ResetPartMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn) {
  std::unique_lock<mutex_t> lock(*(access_));
  unsigned int len = xn - x0;
  for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_)
    memset(costmap_ + y, default_value_, len * sizeof(unsigned char));
}

bool Costmap2D::CopyCostMapWindow(const Costmap2D &map,
                                  double w_origin_x,
                                  double w_origin_y,
                                  double w_size_x,
                                  double w_size_y) {
  if (this == &map)
    return false;
  DeleteMaps();
  unsigned int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  if (!(map.World2Map(w_origin_x, w_origin_y, lower_left_x, lower_left_y)) || !(map.World2Map(w_origin_x + w_size_x, \
     w_origin_y + w_size_y, upper_right_x, upper_right_y)))
    return false;
  size_x_ = upper_right_x - lower_left_x;
  size_y_ = upper_right_y - lower_left_y;
  resolution_ = map.resolution_;
  origin_x_ = w_origin_x;
  origin_y_ = w_origin_y;
  InitMaps(size_x_, size_y_);
  CopyMapRegion(map.costmap_, costmap_, map.size_x_, size_x_, lower_left_x, lower_left_y, 0, 0, size_x_, size_y_);
  return true;
}

Costmap2D &Costmap2D::operator=(const Costmap2D &map) {
  // check for self assignement
  if (this == &map)
    return *this;
  DeleteMaps();
  size_x_ = map.size_x_;
  size_y_ = map.size_y_;
  resolution_ = map.resolution_;
  origin_x_ = map.origin_x_;
  origin_y_ = map.origin_y_;
  InitMaps(size_x_, size_y_);
  memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));
  return *this;
}

Costmap2D::Costmap2D(const Costmap2D &map) :
    costmap_(NULL) {
  access_ = new mutex_t();
  *this = map;
}

Costmap2D::Costmap2D() :
    size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL) {
  access_ = new mutex_t();
}

Costmap2D::~Costmap2D() {
  DeleteMaps();
  delete access_;
}

unsigned int Costmap2D::World2Cell(double world_dist) {
  double cells_dist = std::max(0.0, ceil(world_dist / resolution_));
  return (unsigned int) cells_dist;
}

unsigned char *Costmap2D::GetCharMap() const {
  return costmap_;
}

unsigned char Costmap2D::GetCost(unsigned int mx, unsigned int my) const {
  return costmap_[GetIndex(mx, my)];
}

void Costmap2D::SetCost(unsigned int mx, unsigned int my, unsigned char cost) {
  costmap_[GetIndex(mx, my)] = cost;
}

void Costmap2D::Map2World(unsigned int mx, unsigned int my, double &wx, double &wy) const {
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

bool Costmap2D::World2Map(double wx, double wy, unsigned int &mx, unsigned int &my) const {
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }
  mx = (int) ((wx - origin_x_) / resolution_);
  my = (int) ((wy - origin_y_) / resolution_);
  if (mx < size_x_ && my < size_y_) {
    return true;
  }
  return false;
}

void Costmap2D::World2MapNoBoundary(double world_x,
                                    double world_y,
                                    int &cell_x,
                                    int &cell_y) const {
  cell_x = (int) ((world_x - origin_x_) / resolution_);
  cell_y = (int) ((world_y - origin_y_) / resolution_);
}

void Costmap2D::World2MapWithBoundary(double wx, double wy, int &mx, int &my) const {
  if (wx < origin_x_) {
    mx = 0;
  } else if (wx > resolution_ * size_x_ + origin_x_) {
    mx = size_x_ - 1;
  } else {
    mx = (int) ((wx - origin_x_) / resolution_);
  }

  if (wy < origin_y_) {
    my = 0;
  } else if (wy > resolution_ * size_y_ + origin_y_) {
    my = size_y_ - 1;
  } else {
    my = (int) ((wy - origin_y_) / resolution_);
  }
}

void Costmap2D::UpdateOrigin(double new_origin_x, double new_origin_y) {
  int cell_ox, cell_oy;
  cell_ox = int((new_origin_x - origin_x_) / resolution_);
  cell_oy = int((new_origin_y - origin_y_) / resolution_);
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;
  int size_x = size_x_;
  int size_y = size_y_;
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);
  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;
  unsigned char *local_map = new unsigned char[cell_size_x * cell_size_y];
  CopyMapRegion(costmap_, local_map, size_x_, cell_size_x, lower_left_x, lower_left_y, 0, 0, cell_size_x, cell_size_y);
  ResetMaps();
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;
  CopyMapRegion(local_map, costmap_, cell_size_x, size_x_, 0, 0, start_x, start_y, cell_size_x, cell_size_y);
  delete[] local_map;
}

bool Costmap2D::SetConvexRegionCost(const std::vector<geometry_msgs::Point> &polygon, unsigned char cost_value) {
  std::vector<MapLocation> map_polygon;
  for (unsigned int i = 0; i < polygon.size(); ++i) {
    MapLocation loc;
    if (!World2Map(polygon[i].x, polygon[i].y, loc.x, loc.y)) {
      return false;
    }
    map_polygon.push_back(loc);
  }
  std::vector<MapLocation> polygon_cells;
  FillConvexCells(map_polygon, polygon_cells);
  for (unsigned int i = 0; i < polygon_cells.size(); ++i) {
    unsigned int index = GetIndex(polygon_cells[i].x, polygon_cells[i].y);
    costmap_[index] = cost_value;
  }
  return true;
}

void Costmap2D::GetConvexEdge(const std::vector<MapLocation> &polygon, std::vector<MapLocation> &polygon_cells) {
  PolygonOutlineCells cell_gatherer(*this, costmap_, polygon_cells);
  for (unsigned int i = 0; i < polygon.size() - 1; ++i) {
    RaytraceLine(cell_gatherer, polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y);
  }
  if (!polygon.empty()) {
    unsigned int last_index = polygon.size() - 1;
    RaytraceLine(cell_gatherer, polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y);
  }
}

void Costmap2D::FillConvexCells(const std::vector<MapLocation> &polygon, std::vector<MapLocation> &polygon_cells) {
  if (polygon.size() < 3)
    return;
  GetConvexEdge(polygon, polygon_cells);
  MapLocation swap;
  unsigned int i = 0;
  while (i < polygon_cells.size() - 1) {
    if (polygon_cells[i].x > polygon_cells[i + 1].x) {
      swap = polygon_cells[i];
      polygon_cells[i] = polygon_cells[i + 1];
      polygon_cells[i + 1] = swap;

      if (i > 0)
        --i;
    } else
      ++i;
  }

  i = 0;
  MapLocation min_pt;
  MapLocation max_pt;
  unsigned int min_x = polygon_cells[0].x;
  unsigned int max_x = polygon_cells[polygon_cells.size() - 1].x;
  for (unsigned int x = min_x; x <= max_x; ++x) {
    if (i >= polygon_cells.size() - 1)
      break;

    if (polygon_cells[i].y < polygon_cells[i + 1].y) {
      min_pt = polygon_cells[i];
      max_pt = polygon_cells[i + 1];
    } else {
      min_pt = polygon_cells[i + 1];
      max_pt = polygon_cells[i];
    }

    i += 2;
    while (i < polygon_cells.size() && polygon_cells[i].x == x) {
      if (polygon_cells[i].y < min_pt.y)
        min_pt = polygon_cells[i];
      else if (polygon_cells[i].y > max_pt.y)
        max_pt = polygon_cells[i];
      ++i;
    }

    MapLocation pt;
    for (unsigned int y = min_pt.y; y < max_pt.y; ++y) {
      pt.x = x;
      pt.y = y;
      polygon_cells.push_back(pt);
    }
  }
}

unsigned int Costmap2D::GetSizeXCell() const {
  return size_x_;
}

unsigned int Costmap2D::GetSizeYCell() const {
  return size_y_;
}

double Costmap2D::GetSizeXWorld() const {
  return (size_x_ - 1 + 0.5) * resolution_;
}

double Costmap2D::GetSizeYWorld() const {
  return (size_y_ - 1 + 0.5) * resolution_;
}

double Costmap2D::GetOriginX() const {
  return origin_x_;
}

double Costmap2D::GetOriginY() const {
  return origin_y_;
}

double Costmap2D::GetResolution() const {
  return resolution_;
}

bool Costmap2D::SaveMap(std::string file_name) {
  FILE *fp = fopen(file_name.c_str(), "w");

  if (!fp) {
    return false;
  }
  fprintf(fp, "P2\n%u\n%u\n%u\n", size_x_, size_y_, 0xff);
  for (unsigned int iy = 0; iy < size_y_; iy++) {
    for (unsigned int ix = 0; ix < size_x_; ix++) {
      unsigned char cost = GetCost(ix, iy);
      fprintf(fp, "%d ", cost);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return true;
}

} //namespace roborts_costmap