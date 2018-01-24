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

#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <thread>
#include "messages/GlobalPlannerAction.h"
#include "messages/LocalPlannerAction.h"
#include "messages/ArmorDetectionAction.h"
#include "messages/LocalizationAction.h"

#include "common/error_code.h"
#include "common/io.h"
#include "common/node_state.h"
#include "modules/decision/proto/decision.pb.h"

namespace rrts{
namespace decision{

using rrts::common::ErrorCode;
using rrts::common::ErrorInfo;

class DecisionNode{
 public:
  DecisionNode():global_planner_actionlib_client_("global_planner_node_action", true),
                        local_planner_actionlib_client_("local_planner_node_action",true),
                        localization_actionlib_client_("localization_node_action",true),
                        armor_detection_actionlib_client_("armor_detection_node_action",true),
                        new_goal_(false),new_path_(false),
                        decision_state_(rrts::common::IDLE){
    //point mode
    ros::NodeHandle rviz_nh("move_base_simple");
    goal_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1,
                                                              &DecisionNode::GoalCallback,this);
    LoadParam();
    localization_actionlib_client_.waitForServer();
    std::cout<<"Localization module has been connected!"<<std::endl;
    localization_goal_.command = 1;
    localization_actionlib_client_.sendGoal(localization_goal_);

//    armor_detection_actionlib_client_.waitForServer();
//    std::cout<<"Armor detection module has been connected!"<<std::endl;
//    armor_detection_goal_.command = 1;
//    armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
//                                               actionlib::SimpleActionClient<messages::ArmorDetectionAction>::SimpleDoneCallback(),
//                                               actionlib::SimpleActionClient<messages::ArmorDetectionAction>::SimpleActiveCallback(),
//                                               boost::bind(&DecisionNode::ArmorDetectionFeedbackCallback, this, _1));

    global_planner_actionlib_client_.waitForServer();
    std::cout<<"Global planner module has been connected!"<<std::endl;

    local_planner_actionlib_client_.waitForServer();
    std::cout<<"Local planner module has been connected!"<<std::endl;

    thread_ = std::thread(&DecisionNode::Execution, this);
  }

  void LoadParam() {
    rrts::decision::PatrolPoints patrol_points;
    std::string file_name = "modules/decision/config/decision.prototxt";
    rrts::common::ReadProtoFromTextFile(file_name, &patrol_points);
    unsigned int point_size = patrol_points.point().size();
    patrol_points_.resize(point_size);
    for(unsigned int i = 0; i < point_size; i++) {
      patrol_points_[i].push_back(patrol_points.point(i).x());
      patrol_points_[i].push_back(patrol_points.point(i).y());
      patrol_points_[i].push_back(patrol_points.point(i).z());

      patrol_points_[i].push_back(patrol_points.point(i).roll());
      patrol_points_[i].push_back(patrol_points.point(i).pitch());
      patrol_points_[i].push_back(patrol_points.point(i).yaw());
    }
    //std::sort(patrol_points_.end(), patrol_points_.begin());
  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr & goal){
    global_planner_goal_.goal = *goal;
    new_goal_ = true;
  }

  void Execution(){
    while(ros::ok()) {

      if (new_goal_) {

        global_planner_actionlib_client_.sendGoal(global_planner_goal_,
                     boost::bind(&DecisionNode::GlobalPlannerDoneCallback, this, _1, _2),
                     boost::bind(&DecisionNode::GlobalPlannerActiveCallback, this),
                     boost::bind(&DecisionNode::GlobalPlannerFeedbackCallback, this, _1)
        );
        new_goal_ = false;
      } else if(!patrol_points_.empty() && decision_state_ == rrts::common::IDLE) {

        std::vector<float> point = patrol_points_.back();
        patrol_points_.pop_back();
        global_planner_goal_.goal.header.frame_id = "map";

        global_planner_goal_.goal.pose.position.x = point[0];
        global_planner_goal_.goal.pose.position.y = point[1];
        global_planner_goal_.goal.pose.position.z = point[2];

        tf::Quaternion quaternion =  tf::createQuaternionFromRPY(point[4], point[5], point[6]);
        global_planner_goal_.goal.pose.orientation.w = quaternion.w();
        global_planner_goal_.goal.pose.orientation.x = quaternion.x();
        global_planner_goal_.goal.pose.orientation.y = quaternion.y();
        global_planner_goal_.goal.pose.orientation.z = quaternion.z();

        global_planner_actionlib_client_.sendGoal(global_planner_goal_,
                                                  boost::bind(&DecisionNode::GlobalPlannerDoneCallback, this, _1, _2),
                                                  boost::bind(&DecisionNode::GlobalPlannerActiveCallback, this),
                                                  boost::bind(&DecisionNode::GlobalPlannerFeedbackCallback, this, _1));
        decision_state_ = rrts::common::RUNNING;
      }
      if (new_path_) {

        local_planner_actionlib_client_.sendGoal(local_planner_goal_);
        new_path_ = false;
      }
    }

  }

  // Global Planner
  void GlobalPlannerDoneCallback(const actionlib::SimpleClientGoalState& state,  const messages::GlobalPlannerResultConstPtr& result){
    std::cout<<"Global planner "<<state.toString().c_str()<<"!"<<std::endl;
    decision_state_ = rrts::common::IDLE;
  }
  void GlobalPlannerActiveCallback(){
    std::cout<<"Global planner server has recived the goal!"<<std::endl;
  }
  void GlobalPlannerFeedbackCallback(const messages::GlobalPlannerFeedbackConstPtr& feedback){
    if (feedback->error_code != ErrorCode::OK) {
      std::cout<<"Global planner: "<<feedback->error_msg<<std::endl;
    }
    if (!feedback->path.poses.empty()) {
      local_planner_goal_.route = feedback->path;
      new_path_=true;
    }
  }

  // Armor Detection
  void ArmorDetectionFeedbackCallback(const messages::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->error_code != ErrorCode::OK) {
    }
    else {
      //new_path_=true;
    }
  }

  ~DecisionNode(){
//    localization_goal_.command = '3';
//    localization_actionlib_client_.sendGoal(localization_goal_);
//    armor_detection_goal_.command = '3';
//    armor_detection_actionlib_client_.sendGoal(armor_detection_goal_);
    if(thread_.joinable()){
      thread_.join();
    }

  }
 private:
  actionlib::SimpleActionClient<messages::GlobalPlannerAction> global_planner_actionlib_client_;
  actionlib::SimpleActionClient<messages::LocalPlannerAction> local_planner_actionlib_client_;
  actionlib::SimpleActionClient<messages::LocalizationAction> localization_actionlib_client_;
  actionlib::SimpleActionClient<messages::ArmorDetectionAction> armor_detection_actionlib_client_;

  messages::LocalizationGoal localization_goal_;
  messages::ArmorDetectionGoal armor_detection_goal_;
  messages::GlobalPlannerGoal global_planner_goal_;
  messages::LocalPlannerGoal local_planner_goal_;

  bool new_goal_;
  bool new_path_;

  ros::Subscriber goal_sub_;
  std::thread thread_;

  std::vector<std::vector<float>> patrol_points_;
  rrts::common::NodeState decision_state_;
};

}// namespace decision
}// namespace rrts

int main(int argc, char **argv) {
  ros::init(argc, argv, "decision_node");
  rrts::decision::DecisionNode global_planner_client;
  ros::spin();
  return 0;
}