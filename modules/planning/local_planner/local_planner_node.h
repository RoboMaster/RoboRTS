/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
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

#ifndef MODULES_PLANNING_LOCAL_PLANNER_NODE_H
#define MODULES_PLANNING_LOCAL_PLANNER_NODE_H

#include <string>
#include <thread>
#include <memory>
#include <mutex>
#include <functional>
#include <condition_variable>

#include <nav_msgs/Path.h>
#include <actionlib/server/simple_action_server.h>

#include "common/log.h"
#include "common/rrts.h"
#include "common/io.h"
#include "common/error_code.h"
#include "common/node_state.h"
#include "common/main_interface.h"
#include "common/algorithm_factory.h"

#include "modules/perception/map/costmap/costmap_interface.h"
#include "messages/LocalPlannerAction.h"


#include "modules/planning/local_planner/proto/local_planner.pb.h"
#include "modules/planning/local_planner/local_planner_base.h"
#include "modules/planning/local_planner/local_visualization.h"
#include "modules/planning/local_planner/local_planner_algorithms.h"

namespace rrts {
namespace planning {
namespace local_planner {

class LocalPlannerNode : public rrts::common::RRTS {
 public:
  LocalPlannerNode(std::string name);
  ~LocalPlannerNode();

  rrts::common::ErrorInfo Init();
  void Loop();
  void ExcuteCB(const messages::LocalPlannerGoal::ConstPtr &command);
  void AlgorithmCB(const rrts::common::ErrorInfo &algorithm_error_info);
  void StartPlanning();
  void StopPlanning();

  void SetNodeState(const rrts::common::NodeState& node_state);
  rrts::common::NodeState GetNodeState();

  void SetErrorInfo(const rrts::common::ErrorInfo error_info);
  rrts::common::ErrorInfo GetErrorInfo();

 private:
  ros::NodeHandle local_planner_nh_;
  std::thread local_planner_thread_;
  actionlib::SimpleActionServer<messages::LocalPlannerAction> as_;
  std::unique_ptr<LocalPlannerBase> local_planner_;
  std::mutex node_state_mtx_;
  std::mutex node_error_info_mtx_;
  std::mutex plan_mtx_;
  rrts::common::NodeState node_state_;
  rrts::common::ErrorInfo node_error_info_;
  std::shared_ptr<rrts::perception::map::CostmapInterface> local_cost_;
  std::shared_ptr<tf::TransformListener> tf_;
  bool initialized_;

  std::string selected_algorithm_;

  geometry_msgs::Twist cmd_vel_;
  LocalVisualizationPtr visual_;
  std::string visual_frame_;
  ros::Publisher vel_pub_;
  geometry_msgs::PoseStamped local_goal_;
  int max_error_;
  std::condition_variable plan_condition_;
  std::mutex plan_mutex_;
  double frequency_;

};

} // namespace local_planner
} // namespace planning
} // namespace rrts

#endif //MODULES_PLANNING_LOCAL_PLANNER_NODE_H
