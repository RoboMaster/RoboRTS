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

#include "local_planner/robot_position_cost.h"

namespace roborts_local_planner {

RobotPositionCost::RobotPositionCost(const roborts_costmap::Costmap2D& cost_map) : costmap_(cost_map) {

}

RobotPositionCost::~RobotPositionCost() {

}

double RobotPositionCost::PointCost(int x, int y) {
  unsigned char cost = costmap_.GetCost(x, y);
  if(cost == LETHAL_OBSTACLE || cost == NO_INFORMATION){
    return -1;
  }

  return cost;
}

double RobotPositionCost::LineCost(int x0, int x1, int y0, int y1) {
  double line_cost = 0.0;
  double point_cost = -1.0;

  for(FastLineIterator line( x0, y0, x1, y1 ); line.IsValid(); line.Advance()) {
    point_cost = PointCost(line.GetX(), line.GetY()); //current point's cost

    if(point_cost < 0){
      return -1;
    }

    if(line_cost < point_cost){
      line_cost = point_cost;
    }

  }
  return line_cost;
}

double RobotPositionCost::FootprintCost (const Eigen::Vector2d& position, const std::vector<Eigen::Vector2d>& footprint) {

  unsigned int cell_x, cell_y;

  if (!costmap_.World2Map(position.coeffRef(0), position.coeffRef(1), cell_x, cell_y)) {
    return -1.0;
  }

  if (footprint.size() < 3) {
    unsigned char cost = costmap_.GetCost(cell_x, cell_y);

    if (cost == LETHAL_OBSTACLE  || cost == NO_INFORMATION /*|| cost == INSCRIBED_INFLATED_OBSTACLE*/) {
      return -1.0;
    }
    return cost;
  }

  unsigned int x0, x1, y0, y1;
  double line_cost = 0.0;
  double footprint_cost = 0.0;

  for(unsigned int i = 0; i < footprint.size() - 1; ++i) {
    if (!costmap_.World2Map(footprint[i].coeffRef(0), footprint[i].coeffRef(1), x0, y0)) {
      return -1.0;
    }

    if(!costmap_.World2Map(footprint[i + 1].coeffRef(0), footprint[i + 1].coeffRef(1), x1, y1)) {
      return -1.0;
    }

    line_cost = LineCost(x0, x1, y0, y1);
    footprint_cost = std::max(line_cost, footprint_cost);

    if(line_cost < 0) {
      return -1.0;
    }
  }

  if(!costmap_.World2Map(footprint.back().coeffRef(0), footprint.back().coeffRef(1), x0, y0)) {
    return -1.0;
  }

  if(!costmap_.World2Map(footprint.front().coeffRef(0), footprint.front().coeffRef(1), x1, y1)) {
    return -1.0;
  }

  line_cost = LineCost(x0, x1, y0, y1);
  footprint_cost = std::max(line_cost, footprint_cost);

  if(line_cost < 0) {
    return -1.0;
  }

  return footprint_cost;
}

double RobotPositionCost::FootprintCost(double x,
                                        double y,
                                        double theta,
                                        const std::vector<Eigen::Vector2d> &footprint_spec,
                                        double inscribed_radius,
                                        double circumscribed_radius) {
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  std::vector<Eigen::Vector2d> oriented_footprint;
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    Eigen::Vector2d new_pt;
    new_pt.coeffRef(0) = x + (footprint_spec[i].coeffRef(0) * cos_th - footprint_spec[i].coeffRef(1) * sin_th);
    new_pt.coeffRef(1) = y + (footprint_spec[i].coeffRef(0) * sin_th + footprint_spec[i].coeffRef(1) * cos_th);
    oriented_footprint.push_back(new_pt);
  }

  Eigen::Vector2d robot_position;
  robot_position.coeffRef(0) = x;
  robot_position.coeffRef(1) = y;

  if(inscribed_radius==0.0){
    CalculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);
  }

  return FootprintCost(robot_position, oriented_footprint);
}

} // namespace roborts_local_planner

