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
#ifndef ROBORTS_COSTMAP_COSTMAP_2D_H
#define ROBORTS_COSTMAP_COSTMAP_2D_H

#include <vector>
#include <queue>
#include <mutex>
#include <geometry_msgs/Point.h>

namespace roborts_costmap{

/**
 * @brief A struct representing a map point coordinate
 */
struct MapLocation {
  unsigned int x;
  unsigned int y;
};

/**
 * @class Costmap2D
 * @brief Map provides a mapping between the world and map cost
 */
class Costmap2D {
 public:
  /**
   * @brief  Constructor for a costmap
   * @param  cells_size_x The x size of the map in cells
   * @param  cells_size_y The y size of the map in cells
   * @param  resolution The resolution of the map in meters/cell
   * @param  origin_x The x origin of the map in meters
   * @param  origin_y The y origin of the map in meters
   * @param  default_value Default Value
   */
  Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
            double origin_x, double origin_y, unsigned char default_value = 0);

  /**
   * @brief  Copy constructor for a costmap, creates a copy efficiently
   * @param  map The costmap to copy
   */
  Costmap2D(const Costmap2D &map);

  /**
   * @brief  Overloaded assignment operator
   * @param  map The costmap to copy
   * @return A reference to the map after the copy has finished
   */
  Costmap2D &operator=(const Costmap2D &map);

  /**
   * @brief  Turn this costmap into a copy of a window of a costmap passed in
   * @param  map The costmap to copy
   * @param win_origin_x The x origin (lower left corner) for the window to copy, in meters
   * @param win_origin_y The y origin (lower left corner) for the window to copy, in meters
   * @param win_size_x The x size of the window, in meters
   * @param win_size_y The y size of the window, in meters
   */
  bool CopyCostMapWindow(const Costmap2D &map, double win_origin_x, double win_origin_y, double win_size_x,
                         double win_size_y);

  /**
   * @brief  Default constructor
   */
  Costmap2D();

  /**
   * @brief  Destructor
   */
  virtual ~Costmap2D();

  /**
   * @brief  Get the cost of a cell in the costmap
   * @param  mx The x coordinate of the cell
   * @param  my The y coordinate of the cell
   * @return The cost of the cell
   */
  unsigned char GetCost(unsigned int mx, unsigned int my) const;

  /**
   * @brief  Set the cost of a cell in the costmap
   * @param  mx The x coordinate of the cell
   * @param  my The y coordinate of the cell
   * @param  cost The cost to set the cell to
   */
  void SetCost(unsigned int mx, unsigned int my, unsigned char cost);

  /**
   * @brief  Convert from map coordinates to world coordinates
   * @param  mx The x map coordinate
   * @param  my The y map coordinate
   * @param  wx Will be set to the associated world x coordinate
   * @param  wy Will be set to the associated world y coordinate
   */
  void Map2World(unsigned int mx, unsigned int my, double &wx, double &wy) const;

  /**
   * @brief  Convert from world coordinates to map coordinates
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  mx Will be set to the associated map x coordinate
   * @param  my Will be set to the associated map y coordinate
   * @return True if the conversion was successful (legal bounds) false otherwise
   */
  bool World2Map(double wx, double wy, unsigned int &mx, unsigned int &my) const;

  /**
   * @brief  Convert from world coordinates to map coordinates without checking for legal bounds
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  mx Will be set to the associated map x coordinate
   * @param  my Will be set to the associated map y coordinate
   * @note   The returned map coordinates are not guaranteed to lie within the map.
   */
  void World2MapNoBoundary(double wx, double wy, int &mx, int &my) const;

  /**
   * @brief  Convert from world coordinates to map coordinates, constraining results to legal bounds.
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  mx Will be set to the associated map x coordinate
   * @param  my Will be set to the associated map y coordinate
   * @note   The returned map coordinates are guaranteed to lie in the map.
   */
  void World2MapWithBoundary(double wx, double wy, int &mx, int &my) const;

  /**
   * @brief  Given two map coordinates... compute the associated index
   * @param mx The x coordinate
   * @param my The y coordinate
   * @return The associated index
   */
  inline unsigned int GetIndex(unsigned int mx, unsigned int my) const {
    return my * size_x_ + mx;
  }

  /**
   * @brief  Given an index, compute the associated map coordinates
   * @param  index The index
   * @param  mx Will be set to the x coordinate
   * @param  my Will be set to the y coordinate
   */
  inline void Index2Cells(unsigned int index, unsigned int &mx, unsigned int &my) const {
    my = index / size_x_;
    mx = index - (my * size_x_);
  }

  /**
   * @brief  Will return a pointer to the underlying unsigned char array used as the costmap
   * @return A pointer to the underlying unsigned char array storing cost values
   */
  unsigned char *GetCharMap() const;

  /**
   * @brief  Accessor for the x size of the costmap in cells
   * @return The x size of the costmap in cells
   */
  unsigned int GetSizeXCell() const;

  /**
   * @brief  Accessor for the y size of the costmap in cells
   * @return The y size of the costmap in cells
   */
  unsigned int GetSizeYCell() const;

  /**
   * @brief  Accessor for the x size of the costmap in meters
   * @return The x size of the costmap (returns the centerpoint of the last legal cell in the map)
   */
  double GetSizeXWorld() const;

  /**
   * @brief  Accessor for the y size of the costmap in meters
   * @return The y size of the costmap (returns the centerpoint of the last legal cell in the map)
   */
  double GetSizeYWorld() const;

  /**
   * @brief  Accessor for the x origin of the costmap
   * @return The x origin of the costmap
   */
  double GetOriginX() const;

  /**
   * @brief  Accessor for the y origin of the costmap
   * @return The y origin of the costmap
   */
  double GetOriginY() const;

  /**
   * @brief  Accessor for the resolution of the costmap
   * @return The resolution of the costmap
   */
  double GetResolution() const;
  /**
   * @brief Set the default map of the costmap
   * @param c the cost value to set
   */
  void SetDefaultValue(unsigned char c) {
    default_value_ = c;
  }
  /**
   * @brief Get the default value of the costmap
   * @return  the default value of the map
   */
  unsigned char GetDefaultValue() const {
    return default_value_;
  }

  /**
   * @brief  Sets the cost of a convex polygon to a desired value
   * @param  polygon The polygon to perform the operation on
   * @param  cost_value The value to set costs to
   * @return True if the polygon was filled, false if it could not be filled
   */
  bool SetConvexRegionCost(const std::vector<geometry_msgs::Point> &polygon_edge_world, unsigned char value);

  /**polygonOutlineCells
   * @brief  Get the map cells that make up the outline of a polygon
   * @param convex_region_cells The polygon in map coordinates to rasterize
   * @param convex_edge_cells Will be set to the cells contained in the outline of the polygon
   */

  void GetConvexEdge(const std::vector<MapLocation> &convex_region_cells, std::vector<MapLocation> &convex_edge_cells);

  /**convexFillCells
   * @brief  Get the map cells that fill a convex polygon
   * @param convex_edge_cells The polygon in map coordinates to rasterize
   * @param convex_region_cells Will be set to the cells that fill the polygon
   */
  void FillConvexCells(const std::vector<MapLocation> &convex_edge_cells,
                       std::vector<MapLocation> &convex_region_cells);

  /**
   * @brief  Move the origin of the costmap to a new location.... keeping data when it can
   * @param  new_origin_x The x coordinate of the new origin
   * @param  new_origin_y The y coordinate of the new origin
   */
  virtual void UpdateOrigin(double new_origin_x, double new_origin_y);

  /**
   * @brief  Save the costmap out to a pgm file
   * @param file_name The name of the file to save
   */
  bool SaveMap(std::string map_name);

  /**
   * @brief Reset the size of the map
   * @param size_x The new size x to set
   * @param size_y The new size y to set
   * @param resolution The new resolution to set
   * @param origin_x The new origin x to set
   * @param origin_y The new origin y to set
   */
  void ResizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                 double origin_y);

  void ResetPartMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);

  /**
   * @brief  Given distance in the world, convert it to cells
   * @param  world_dist The world distance
   * @return The equivalent cell distance
   */
  unsigned int World2Cell(double world_dist);

  typedef std::recursive_mutex mutex_t;
  mutex_t *GetMutex() {
    return access_;
  }

 protected:

/**
 * @brief  Copy a region of a source map into a destination map
 * @tparam maptype
 * @param src_map The source map
 * @param dst_map The destination map
 * @param src_map_size_x The size X of the src
 * @param dst_map_size_x The size X of the dst
 * @param src_map_lower_left_x_start_to_copy The start point to copy in src x
 * @param src_map_lower_left_y_start_to_copy The start point to copy in src y
 * @param dst_map_lower_left_x_start_to_copy The start point to copy in dst x
 * @param dst_map_lower_left_y_start_to_copy The start point to copy in dst y
 * @param copy_region_size_x The copy window size X
 * @param copy_region_size_y The copy window size Y
 */
  template<typename maptype>
  void CopyMapRegion(maptype *src_map,
                     maptype *dst_map, \

                     unsigned int src_map_size_x,
                     unsigned int dst_map_size_x, \

                     unsigned int src_map_lower_left_x_start_to_copy,
                     unsigned int src_map_lower_left_y_start_to_copy, \

                     unsigned int dst_map_lower_left_x_start_to_copy,
                     unsigned int dst_map_lower_left_y_start_to_copy, \

                     unsigned int copy_region_size_x,
                     unsigned int copy_region_size_y) {
    maptype *src_copy_ptr =
        src_map + (src_map_lower_left_x_start_to_copy + src_map_lower_left_y_start_to_copy * src_map_size_x);
    maptype *dst_copy_ptr =
        dst_map + (dst_map_lower_left_x_start_to_copy + dst_map_lower_left_y_start_to_copy * dst_map_size_x);
    for (unsigned i = 0; i < copy_region_size_y; ++i) {
      memcpy(dst_copy_ptr, src_copy_ptr, copy_region_size_x * sizeof(maptype));
      dst_copy_ptr += dst_map_size_x;
      src_copy_ptr += src_map_size_x;
    }
  }

  /**
   * @brief  Deletes the costmap, static_map, and markers data structures
   */
  virtual void DeleteMaps();

  /**
   * @brief  Reset the costmap and static_map to be unknown space
   */
  virtual void ResetMaps();

  /**
   * @brief  Initialize the costmap, static_map, and markers data structures
   * @param size_x The x size to use for map initialization
   * @param size_y The y size to use for map initialization
   */
  virtual void InitMaps(unsigned int size_x, unsigned int size_y);

  /**
   * @brief  Raytrace a line and apply some action at each step
   * @param  at The action to take... a functor
   * @param  x0 The starting x coordinate
   * @param  y0 The starting y coordinate
   * @param  x1 The ending x coordinate
   * @param  y1 The ending y coordinate
   * @param  max_length The maximum desired length of the segment.allowing to not go all the way to the endpoint
   */
  template<typename ActionType>
  inline void RaytraceLine(ActionType at, unsigned int x0, unsigned int y0, \
                                                         unsigned int x1, unsigned int y1, \
                           unsigned int max_length = UINT_MAX) {
    int dx = x1 - x0;
    int dy = y1 - y0;
    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);
    int offset_dx = Sign(dx);
    int offset_dy = Sign(dy) * size_x_;
    unsigned int offset = y0 * size_x_ + x0;
    double dist = hypot(dx, dy);
    double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);
    // if x is dominant
    if (abs_dx >= abs_dy) {
      int error_y = abs_dx / 2;
      Bresenham(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int) (scale * abs_dx));
      return;
    }
    // otherwise y is dominant
    int error_x = abs_dy / 2;
    Bresenham(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int) (scale * abs_dy));
  }

 private:
  /**
   * @brief  A 2D implementation of Bresenham's raytracing algorithm... applies an action at each step
   */
  template<typename ActionType>
  inline void Bresenham(ActionType at, unsigned int abs_da, unsigned int abs_db, \
                                                      int error_b, int offset_a, \
                                                      int offset_b, unsigned int offset, unsigned int max_length) {
    unsigned int end = std::min(max_length, abs_da);
    for (unsigned int i = 0; i < end; ++i) {
      at(offset);
      offset += offset_a;
      error_b += abs_db;
      if ((unsigned int) error_b >= abs_da) {
        offset += offset_b;
        error_b -= abs_da;
      }
    }
    at(offset);
  }

  inline int Sign(int x) {
    return x > 0 ? 1 : -1;
  }

  mutex_t *access_;
 protected:
  unsigned int size_x_;
  unsigned int size_y_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  unsigned char *costmap_;
  unsigned char default_value_;

  class MarkCell {
   public:
    MarkCell(unsigned char *costmap, unsigned char value) :
        costmap_(costmap), value_(value) {}
    inline void operator()(unsigned int offset) {
      costmap_[offset] = value_;
    }
   private:
    unsigned char *costmap_;
    unsigned char value_;
  };

  class PolygonOutlineCells {
   public:
    PolygonOutlineCells(const Costmap2D &costmap, const unsigned char *char_map, std::vector<MapLocation> &cells) :
        costmap_(costmap), char_map_(char_map), cells_(cells) {
    }

    // just push the relevant cells back onto the list
    inline void operator()(unsigned int offset) {
      MapLocation loc;
      costmap_.Index2Cells(offset, loc.x, loc.y);
      cells_.push_back(loc);
    }

   private:
    const Costmap2D &costmap_;
    const unsigned char *char_map_;
    std::vector<MapLocation> &cells_;
  };
};
} // namespace roborts_costmap

#endif  //ROBORTS_COSTMAP_COSTMAP_2D_H
