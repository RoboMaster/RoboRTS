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

#ifndef ROBORTS_PLANNING_LOCAL_PLANNER_NODE_H
#define ROBORTS_PLANNING_LOCAL_PLANNER_NODE_H

#include <string>
#include <thread>
#include <memory>
#include <mutex>
#include <functional>
#include <condition_variable>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <actionlib/server/simple_action_server.h>

#include "io/io.h"
#include "state/error_code.h"
#include "state/node_state.h"
#include "alg_factory/algorithm_factory.h"

#include "costmap/costmap_interface.h"
#include "roborts_msgs/LocalPlannerAction.h"
#include "roborts_msgs/TwistAccel.h"


#include "local_planner/proto/local_planner.pb.h"
#include "local_planner/local_planner_base.h"
#include "local_planner/local_visualization.h"
#include "local_planner/local_planner_algorithms.h"

namespace roborts_local_planner {

/**
 * @brief Local planner node class
 */
class LocalPlannerNode {
 public:
  /**
   * @brief Constructor
   */
  LocalPlannerNode();
  ~LocalPlannerNode();

  /**
   * @brief init all param
   * @return Error info
   */
  roborts_common::ErrorInfo Init();

  /**
   * @brief Main loop
   */
  void Loop();

  /**
   * @brief Actionlib callback function use to control loop function
   * @param command Command to control loop function
   */
  void ExcuteCB(const roborts_msgs::LocalPlannerGoal::ConstPtr &command);

  /**
   * @brief local planner algorithm's error callback function
   * @param algorithm_error_info local planner algorithm's error info
   */
  void AlgorithmCB(const roborts_common::ErrorInfo &algorithm_error_info);

  /**
   * @brief start local planner algorithm
   */
  void StartPlanning();

  /**
   * @brief stop local planner algorithm
   */
  void StopPlanning();

  /**
   * @brief Set local planner node state
   * @param node_state State want to set
   */
  void SetNodeState(const roborts_common::NodeState& node_state);

  /**
   * @brief Get local planner node state
   * @return State of local planner node
   */
  roborts_common::NodeState GetNodeState();

  /**
   * @brief Set Error info if error occur
   * @param error_info error info
   */
  void SetErrorInfo(const roborts_common::ErrorInfo error_info);

  /**
   * Get error info
   * @return Error in local planner node
   */
  roborts_common::ErrorInfo GetErrorInfo();

 private:

   //! ros node handle
  ros::NodeHandle local_planner_nh_;
  //! local planner algorithm thread
  std::thread local_planner_thread_;
  //! local planner node actionlib server
  actionlib::SimpleActionServer<roborts_msgs::LocalPlannerAction> as_;
  //! local planner algorithm parent pointer
  std::unique_ptr<LocalPlannerBase> local_planner_;
  //! node state mutex
  std::mutex node_state_mtx_;
  //! node error info mutex
  std::mutex node_error_info_mtx_;
  //! planner algorithm mutex
  std::mutex plan_mtx_;
  //! node state
  roborts_common::NodeState node_state_;
  //! error info
  roborts_common::ErrorInfo node_error_info_;
  //! local cost map
  std::shared_ptr<roborts_costmap::CostmapInterface> local_cost_;
  //! tf pointer
  std::shared_ptr<tf::TransformListener> tf_;
  //! initialize state
  bool initialized_;

  //! local planner algorithm which choose to run
  std::string selected_algorithm_;

  //geometry_msgs::Twist cmd_vel_;
  //! robot control velocity with accelerate
  roborts_msgs::TwistAccel cmd_vel_;
  //! visualization ptr
  LocalVisualizationPtr visual_;
  //! frame to visualization
  std::string visual_frame_;
  //! ros publisher
  ros::Publisher vel_pub_;
  //! When no global planner give the global plan, use local goal express robot end point
  geometry_msgs::PoseStamped local_goal_;
  //! local planner algorithm max error
  int max_error_;
  //! local planner condition variable
  std::condition_variable plan_condition_;
  //! local planner mutex
  std::mutex plan_mutex_;
  //! control frequency
  double frequency_;

};

} // namespace roborts_local_planner

#endif //ROBORTS_PLANNING_LOCAL_PLANNER_NODE_H
