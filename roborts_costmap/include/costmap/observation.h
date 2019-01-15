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
#ifndef ROBORTS_COSTMAP_OBSERVATION_H
#define ROBORTS_COSTMAP_OBSERVATION_H
#include <geometry_msgs/Point.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace roborts_costmap {

/**
 * @brief Stores an observation in terms of a point cloud and the origin of the source
 *
 */
class Observation
{
 public:
  /**
   * @brief  Creates an empty observation
   */
  Observation() :
      cloud_(new pcl::PointCloud<pcl::PointXYZ>()), obstacle_range_(0.0), raytrace_range_(0.0)
  {
  }

  virtual ~Observation()
  {
    delete cloud_;
  }

  /**
   * @brief  Creates an observation from an origin point and a point cloud
   * @param  origin The origin point of the observation
   * @param  cloud The point cloud of the observation
   * @param  obstacle_range The range out to which an observation should be able to insert obstacles
   * @param  raytrace_range The range out to which an observation should be able to clear via raytracing
   */
  Observation(geometry_msgs::Point& origin, pcl::PointCloud<pcl::PointXYZ> cloud,
              double obstacle_range, double raytrace_range) :
      origin_(origin), cloud_(new pcl::PointCloud<pcl::PointXYZ>(cloud)),
      obstacle_range_(obstacle_range), raytrace_range_(raytrace_range)
  {
  }

  /**
   * @brief  Copy constructor
   * @param  obs The observation to copy
   */
  Observation(const Observation& obs) :
      origin_(obs.origin_), cloud_(new pcl::PointCloud<pcl::PointXYZ>(*(obs.cloud_))),
      obstacle_range_(obs.obstacle_range_), raytrace_range_(obs.raytrace_range_)
  {
  }

  /**
   * @brief  Creates an observation from a point cloud
   * @param  cloud The point cloud of the observation
   * @param  obstacle_range The range out to which an observation should be able to insert obstacles
   */
  Observation(pcl::PointCloud<pcl::PointXYZ> cloud, double obstacle_range) :
      cloud_(new pcl::PointCloud<pcl::PointXYZ>(cloud)), obstacle_range_(obstacle_range), raytrace_range_(0.0)
  {
  }

  geometry_msgs::Point origin_;
  pcl::PointCloud<pcl::PointXYZ>* cloud_;
  double obstacle_range_, raytrace_range_;
};

}// namespace roborts_costmap
#endif // ROBORTS_COSTMAP_OBSERVATION_H