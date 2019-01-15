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

#include "a_star_planner.h"

namespace roborts_global_planner{

using roborts_common::ErrorCode;
using roborts_common::ErrorInfo;

AStarPlanner::AStarPlanner(CostmapPtr costmap_ptr) :
    GlobalPlannerBase::GlobalPlannerBase(costmap_ptr),
    gridmap_width_(costmap_ptr_->GetCostMap()->GetSizeXCell()),
    gridmap_height_(costmap_ptr_->GetCostMap()->GetSizeYCell()),
    cost_(costmap_ptr_->GetCostMap()->GetCharMap()) {

  AStarPlannerConfig a_star_planner_config;
  std::string full_path = ros::package::getPath("roborts_planning") + "/global_planner/a_star_planner/"\
      "config/a_star_planner_config.prototxt";

  if (!roborts_common::ReadProtoFromTextFile(full_path.c_str(),
                                           &a_star_planner_config)) {
    ROS_ERROR("Cannot load a star planner protobuf configuration file.");
  }
  //  AStarPlanner param config
  heuristic_factor_ = a_star_planner_config.heuristic_factor();
  inaccessible_cost_ = a_star_planner_config.inaccessible_cost();
  goal_search_tolerance_ = a_star_planner_config.goal_search_tolerance()/costmap_ptr->GetCostMap()->GetResolution();
}

AStarPlanner::~AStarPlanner(){
  cost_ = nullptr;
}

ErrorInfo AStarPlanner::Plan(const geometry_msgs::PoseStamped &start,
                             const geometry_msgs::PoseStamped &goal,
                             std::vector<geometry_msgs::PoseStamped> &path) {

  unsigned int start_x, start_y, goal_x, goal_y, tmp_goal_x, tmp_goal_y;
  unsigned int valid_goal[2];
  unsigned  int shortest_dist = std::numeric_limits<unsigned int>::max();
  bool goal_valid = false;

  if (!costmap_ptr_->GetCostMap()->World2Map(start.pose.position.x,
                                             start.pose.position.y,
                                             start_x,
                                             start_y)) {
    ROS_WARN("Failed to transform start pose from map frame to costmap frame");
    return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                     "Start pose can't be transformed to costmap frame.");
  }
  if (!costmap_ptr_->GetCostMap()->World2Map(goal.pose.position.x,
                                             goal.pose.position.y,
                                             goal_x,
                                             goal_y)) {
    ROS_WARN("Failed to transform goal pose from map frame to costmap frame");
    return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                     "Goal pose can't be transformed to costmap frame.");
  }
  if (costmap_ptr_->GetCostMap()->GetCost(goal_x,goal_y)<inaccessible_cost_){
    valid_goal[0] = goal_x;
    valid_goal[1] = goal_y;
    goal_valid = true;
  }else{
    tmp_goal_x = goal_x;
    tmp_goal_y = goal_y - goal_search_tolerance_;

    while(tmp_goal_y <= goal_y + goal_search_tolerance_){
      tmp_goal_x = goal_x - goal_search_tolerance_;
      while(tmp_goal_x <= goal_x + goal_search_tolerance_){
        unsigned char cost = costmap_ptr_->GetCostMap()->GetCost(tmp_goal_x, tmp_goal_y);
        unsigned int dist = abs(goal_x - tmp_goal_x) + abs(goal_y - tmp_goal_y);
        if (cost < inaccessible_cost_ && dist < shortest_dist ) {
          shortest_dist = dist;
          valid_goal[0] = tmp_goal_x;
          valid_goal[1] = tmp_goal_y;
          goal_valid = true;
        }
        tmp_goal_x += 1;
      }
      tmp_goal_y += 1;
    }
  }
  ErrorInfo error_info;
  if (!goal_valid){
    error_info=ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR);
    path.clear();
  }
  else{
    unsigned int start_index, goal_index;
    start_index = costmap_ptr_->GetCostMap()->GetIndex(start_x, start_y);
    goal_index = costmap_ptr_->GetCostMap()->GetIndex(valid_goal[0], valid_goal[1]);

    costmap_ptr_->GetCostMap()->SetCost(start_x, start_y,roborts_costmap::FREE_SPACE);

    if(start_index == goal_index){
      error_info=ErrorInfo::OK();
      path.clear();
      path.push_back(start);
      path.push_back(goal);
    }
    else{
      error_info = SearchPath(start_index, goal_index, path);
      if ( error_info.IsOK() ){
        path.back().pose.orientation = goal.pose.orientation;
        path.back().pose.position.z = goal.pose.position.z;
      }
    }

  }


  return error_info;
}

ErrorInfo AStarPlanner::SearchPath(const int &start_index,
                                   const int &goal_index,
                                   std::vector<geometry_msgs::PoseStamped> &path) {

  g_score_.clear();
  f_score_.clear();
  parent_.clear();
  state_.clear();
  gridmap_width_ = costmap_ptr_->GetCostMap()->GetSizeXCell();
  gridmap_height_ = costmap_ptr_->GetCostMap()->GetSizeYCell();
  ROS_INFO("Search in a map %d", gridmap_width_*gridmap_height_);
  cost_ = costmap_ptr_->GetCostMap()->GetCharMap();
  g_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
  f_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
  parent_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
  state_.resize(gridmap_height_ * gridmap_width_, SearchState::NOT_HANDLED);

  std::priority_queue<int, std::vector<int>, Compare> openlist;
  g_score_.at(start_index) = 0;
  openlist.push(start_index);

  std::vector<int> neighbors_index;
  int current_index, move_cost, h_score, count = 0;

  while (!openlist.empty()) {
    current_index = openlist.top();
    openlist.pop();
    state_.at(current_index) = SearchState::CLOSED;

    if (current_index == goal_index) {
      ROS_INFO("Search takes %d cycle counts", count);
      break;
    }

    GetNineNeighbors(current_index, neighbors_index);

    for (auto neighbor_index : neighbors_index) {

      if (neighbor_index < 0 ||
          neighbor_index >= gridmap_height_ * gridmap_width_) {
        continue;
      }

      if (cost_[neighbor_index] >= inaccessible_cost_ ||
          state_.at(neighbor_index) == SearchState::CLOSED) {
        continue;
      }

      GetMoveCost(current_index, neighbor_index, move_cost);

      if (g_score_.at(neighbor_index) > g_score_.at(current_index) + move_cost + cost_[neighbor_index]) {

        g_score_.at(neighbor_index) = g_score_.at(current_index) + move_cost + cost_[neighbor_index];
        parent_.at(neighbor_index) = current_index;

        if (state_.at(neighbor_index) == SearchState::NOT_HANDLED) {
          GetManhattanDistance(neighbor_index, goal_index, h_score);
          f_score_.at(neighbor_index) = g_score_.at(neighbor_index) + h_score;
          openlist.push(neighbor_index);
          state_.at(neighbor_index) = SearchState::OPEN;
        }
      }
    }
    count++;
  }

  if (current_index != goal_index) {
    ROS_WARN("Global planner can't search the valid path!");
    return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR, "Valid global path not found.");
  }

  unsigned int iter_index = current_index, iter_x, iter_y;

  geometry_msgs::PoseStamped iter_pos;
  iter_pos.pose.orientation.w = 1;
  iter_pos.header.frame_id = "map";
  path.clear();
  costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
  costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
  path.push_back(iter_pos);

  while (iter_index != start_index) {
    iter_index = parent_.at(iter_index);
//    if(cost_[iter_index]>= inaccessible_cost_){
//      LOG_INFO<<"Cost changes through planning for"<< static_cast<unsigned int>(cost_[iter_index]);
//    }
    costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
    costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
    path.push_back(iter_pos);
  }

  std::reverse(path.begin(),path.end());

  return ErrorInfo(ErrorCode::OK);

}

ErrorInfo AStarPlanner::GetMoveCost(const int &current_index,
                                    const int &neighbor_index,
                                    int &move_cost) const {
  if (abs(neighbor_index - current_index) == 1 ||
      abs(neighbor_index - current_index) == gridmap_width_) {
    move_cost = 10;
  } else if (abs(neighbor_index - current_index) == (gridmap_width_ + 1) ||
      abs(neighbor_index - current_index) == (gridmap_width_ - 1)) {
    move_cost = 14;
  } else {
    return ErrorInfo(ErrorCode::GP_MOVE_COST_ERROR,
                     "Move cost can't be calculated cause current neighbor index is not accessible");
  }
  return ErrorInfo(ErrorCode::OK);
}

void AStarPlanner::GetManhattanDistance(const int &index1, const int &index2, int &manhattan_distance) const {
  manhattan_distance = heuristic_factor_* 10 * (abs(index1 / gridmap_width_ - index2 / gridmap_width_) +
      abs(index1 % gridmap_width_ - index2 % gridmap_width_));
}

void AStarPlanner::GetNineNeighbors(const int &current_index, std::vector<int> &neighbors_index) const {
  neighbors_index.clear();
  if(current_index - gridmap_width_ >= 0){
    neighbors_index.push_back(current_index - gridmap_width_);       //up
  }
  if(current_index - gridmap_width_ - 1 >= 0 && (current_index - gridmap_width_ - 1 + 1) % gridmap_width_!= 0){
    neighbors_index.push_back(current_index - gridmap_width_ - 1); //left_up
  }
  if(current_index - 1 >= 0 && (current_index - 1 + 1) % gridmap_width_!= 0){
    neighbors_index.push_back(current_index - 1);        //left
  }
  if(current_index + gridmap_width_ - 1 < gridmap_width_* gridmap_height_
      && (current_index + gridmap_width_ - 1 + 1) % gridmap_width_!= 0){
    neighbors_index.push_back(current_index + gridmap_width_ - 1); //left_down
  }
  if(current_index + gridmap_width_ < gridmap_width_* gridmap_height_){
    neighbors_index.push_back(current_index + gridmap_width_);     //down
  }
  if(current_index + gridmap_width_ + 1 < gridmap_width_* gridmap_height_
      && (current_index + gridmap_width_ + 1 ) % gridmap_width_!= 0){
    neighbors_index.push_back(current_index + gridmap_width_ + 1); //right_down
  }
  if(current_index  + 1 < gridmap_width_* gridmap_height_
      && (current_index  + 1 ) % gridmap_width_!= 0) {
    neighbors_index.push_back(current_index + 1);                   //right
  }
  if(current_index - gridmap_width_ + 1 >= 0
      && (current_index - gridmap_width_ + 1 ) % gridmap_width_!= 0) {
    neighbors_index.push_back(current_index - gridmap_width_ + 1); //right_up
  }
}

} //namespace roborts_global_planner
