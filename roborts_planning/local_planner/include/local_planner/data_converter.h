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

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_DATA_CONVERTER_H_
#define ROBORTS_PLANNING_LOCAL_PLANNER_DATA_CONVERTER_H_

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>

#include "costmap/costmap_interface.h"

/**
 * @brief Convert Data to Eigen, all function is static
 * @usage DataConverter::FUNCTION(param, param, ...)
 * @return std::pair<Eigen::Vector2d, double> or Eigen::Vector2d
 */
namespace roborts_local_planner {
class DataConverter {
 public:
  /**
   * @brief Convert roborts_costmap::RobotPose to std::pair<Eigen::Vector2d, double>
   * @param pose Data structure of roborts_costmap::RobotPose
   * @return Data structure of std::pair<Eigen::Vector2d, double>
   */
  static std::pair<Eigen::Vector2d, double> LocalConvertRMData(const roborts_costmap::RobotPose &pose) {
    Eigen::Vector2d position;
    position.coeffRef(0) = pose.position.coeffRef(0);
    position.coeffRef(1) = pose.position.coeffRef(1);
    Eigen::Vector3f euler = pose.rotation.eulerAngles(2, 1, 0);
    return std::make_pair(position, euler(0, 0));
  }

  /**
   * @brief Convert tf::Pose to std::pair<Eigen::Vector2d, double>
   * @param pose Type of tf::Pose
   * @return Type of std::pair<Eigen::Vector2d, double>
   */
  static std::pair<Eigen::Vector2d, double> LocalConvertTFData(const tf::Pose &pose) {
    Eigen::Vector2d position;
    position.coeffRef(0) = pose.getOrigin().getX();
    position.coeffRef(1) = pose.getOrigin().getY();
    return std::make_pair(position, tf::getYaw(pose.getRotation()));
  }

  /**
   * @brief Convert geometry_msgs::Pose to std::pair<Eigen::Vector2d, double>
   * @param pose Type of geometry_msgs::Pose
   * @return Type of std::pair<Eigen::Vector2d, double>
   */
  static std::pair<Eigen::Vector2d, double> LocalConvertGData(const geometry_msgs::Pose &pose) {
    Eigen::Vector2d position;
    position.coeffRef(0) = pose.position.x;
    position.coeffRef(1) = pose.position.y;
    return std::make_pair(position, tf::getYaw(pose.orientation));
  }

  /**
   * @brief Convert geometry_msgs::Point to Eigen::Vector2d
   * @param point Type of geometry_msgs::Point
   * @return Type of Eigen::Vector2d
   */
  static Eigen::Vector2d LocalConvertGData(const geometry_msgs::Point &point) {
    Eigen::Vector2d position;
    position.coeffRef(0) = point.x;
    position.coeffRef(1) = point.y;
    return position;
  }

  /**
   * @brief Convert std::vector<geometry_msgs::Point> to Eigen::Vector2d
   * @param points vector of geometry_msgs::Point
   * @return vector of Eigen::Vector2d
   */
  static std::vector<Eigen::Vector2d> LocalConvertGData(const std::vector<geometry_msgs::Point> &points) {
    std::vector<Eigen::Vector2d> positions;
    for (int i = 0; i < points.size(); ++i) {
      Eigen::Vector2d temp_point;
      temp_point = LocalConvertGData (points[i]);
      positions.push_back(temp_point);
    }
    return positions;
  }

  /**
   * Convert common type to Eigen::Vector2d
   * @param x Position x
   * @param y Position y
   * @param theta Orientation theta
   * @return std::pair<Eigen::Vector2d, double>
   */
  static std::pair<Eigen::Vector2d, double> LocalConvertCData(double x, double y, double theta) {
    Eigen::Vector2d position;
    position.coeffRef(0) = x;
    position.coeffRef(1) = y;
    return std::make_pair(position, theta);
  }
};


} // namespace roborts_local_planner
#endif //ROBORTS_PLANNING_LOCAL_PLANNER_DATA_CONVERTER_H_

