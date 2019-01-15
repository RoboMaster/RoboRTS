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
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *   * Neither the name of the institute nor the names of its
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
 * Author: Christoph Rösmann
 *********************************************************************/

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include "timed_elastic_band/teb_optimal.h"
#include "timed_elastic_band/proto/timed_elastic_band.pb.h"

#include "local_planner/optimal_base.h"
#include "local_planner/obstacle.h"
#include "local_planner/local_visualization.h"
#include "local_planner/robot_footprint_model.h"



using namespace roborts_local_planner;

// ============= Global Variables ================
OptimalBasePtr planner;
std::vector<ObstaclePtr> obst_vector;
LocalVisualizationPtr visual;
ViaPointContainer via_points;
unsigned int no_fixed_obstacles;

// =========== Function declarations =============
void CB_mainCycle(const ros::TimerEvent& e);
void CB_publishCycle(const ros::TimerEvent& e);
void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame,
                             interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);


// =============== Main function =================
int main( int argc, char** argv ) {
  ros::init(argc, argv, "test_optim_node");
  ros::NodeHandle nh;

  roborts_local_planner::Config param_config_;

  std::string full_path = ros::package::getPath("roborts_planning") + \
      "/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt";
  roborts_common::ReadProtoFromTextFile(full_path.c_str(), &param_config_);
  ros::Timer cycle_timer = nh.createTimer(ros::Duration(0.025), CB_mainCycle);
  ros::Timer publish_timer = nh.createTimer(ros::Duration(0.1), CB_publishCycle);

  interactive_markers::InteractiveMarkerServer marker_server("marker_obstacles");

  obst_vector.emplace_back( boost::make_shared<PointObstacle>(-3,1) );
  obst_vector.emplace_back( boost::make_shared<PointObstacle>(6,2) );
  obst_vector.emplace_back( boost::make_shared<PointObstacle>(0,0.1) );
  obst_vector.emplace_back( boost::make_shared<PointObstacle>(-4,1) );
  obst_vector.emplace_back( boost::make_shared<PointObstacle>(5,2) );
  obst_vector.emplace_back( boost::make_shared<PointObstacle>(1,0.1) );
  obst_vector.emplace_back( boost::make_shared<PointObstacle>(-3,2) );
  obst_vector.emplace_back( boost::make_shared<PointObstacle>(5,3) );
  obst_vector.emplace_back( boost::make_shared<PointObstacle>(4,0) );
  obst_vector.emplace_back( boost::make_shared<PointObstacle>(4,1) );
  obst_vector.emplace_back( boost::make_shared<PointObstacle>(3,2) );
  obst_vector.emplace_back( boost::make_shared<PointObstacle>(2,2) );

  std::string map_frame;
  for (unsigned int i=0; i<obst_vector.size(); ++i) {

    boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(obst_vector.at(i));
    if (pobst) {
      CreateInteractiveMarker(pobst->Position().coeff(0),pobst->Position().coeff(1),
                              i, "odom", &marker_server, &CB_obstacle_marker);
    }
  }
  marker_server.applyChanges();
  
  // Setup visualization
  visual = LocalVisualizationPtr(new LocalVisualization(nh, "odom"));

  RobotFootprintModelPtr model = boost::make_shared<PointRobotFootprint>();

  planner = OptimalBasePtr(new TebOptimal(param_config_ ,&obst_vector, model, visual, &via_points));
  

  no_fixed_obstacles = (unsigned int)obst_vector.size();
  ros::spin();

  return 0;
}

// Planning loop
void CB_mainCycle(const ros::TimerEvent& e) {
  auto start_pose = DataConverter::LocalConvertCData(-4, 0, 0);
  auto end_pose = DataConverter::LocalConvertCData(4, 0, 0);
  planner->Optimal(DataBase(start_pose.first, start_pose.second), DataBase(end_pose.first, end_pose.second));
}

// Visualization loop
void CB_publishCycle(const ros::TimerEvent& e) {
  planner->Visualize();
}

void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame,
                             interactive_markers::InteractiveMarkerServer* marker_server,
                             interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb) {
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker i_marker;
  i_marker.header.frame_id = frame;
  i_marker.header.stamp = ros::Time::now();
  std::ostringstream oss;
  //oss << "obstacle" << id;
  oss << id;
  i_marker.name = oss.str();
  i_marker.description = "Obstacle";
  i_marker.pose.position.x = init_x;
  i_marker.pose.position.y = init_y;

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.id = id;
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 100;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = 1;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  i_marker.controls.push_back( box_control );

  // create a control which will move the box, rviz will insert 2 arrows
  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = sqrt(2) / 2;
  move_control.orientation.x = 0;
  move_control.orientation.y = sqrt(2) / 2;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;


  // add the control to the interactive marker
  i_marker.controls.push_back(move_control);

  // add the interactive marker to our collection
  marker_server->insert(i_marker);
  marker_server->setCallback(i_marker.name, feedback_cb);
}

void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  std::stringstream ss(feedback->marker_name);
  unsigned int index;
  ss >> index;
  if (index>=no_fixed_obstacles)
    return;
  PointObstacle* pobst = dynamic_cast<PointObstacle*>(obst_vector.at(index).get());
  pobst->Position() = Eigen::Vector2d(feedback->pose.position.x,feedback->pose.position.y);
}