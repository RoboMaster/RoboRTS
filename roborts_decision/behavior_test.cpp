#include <ros/ros.h>

#include "executor/chassis_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "blackboard/blackboard.h"
#include "std_msgs/String.h"
#include "behavior_tree/behavior_state.h"
#include "proto/decision.pb.h"
#include <sstream>

void Command();
void goalrccallback(const geometry_msgs::Pose::ConstPtr& msg);
void twistrccallback(const geometry_msgs::Twist::ConstPtr& msg);
void omegarccallback(const roborts_msgs::TwistAccel::ConstPtr& msg);

bool enable_flag = false;
char command = '0';
geometry_msgs::PoseStamped nextGoal;
geometry_msgs::Twist       twist;
roborts_msgs::TwistAccel   omega;
roborts_decision::ChassisExecutor *chassis_executor;
int main(int argc, char **argv) {
  ros::init(argc, argv, "behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  chassis_executor = new roborts_decision::ChassisExecutor();
  auto blackboard = new roborts_decision::Blackboard(full_path);
  // Topics to be published
  ros::NodeHandle n;
  ros::Publisher enemy_pose = n.advertise<geometry_msgs::PoseStamped>("Decision/EnemyPose", 1000);
  ros::Publisher self_pose  = n.advertise<geometry_msgs::PoseStamped>("Decision/SelfPose",  1000);
  ros::Publisher goal       = n.advertise<geometry_msgs::PoseStamped>("Decision/Goal",      1000);
  ros::Publisher is_detected= n.advertise<std_msgs::String>("Decision/IsEnemy",             1000);
  ros::Publisher is_newgoal = n.advertise<std_msgs::String>("Decision/IsNewGoal",           1000);
  ros::Publisher chassis_status = n.advertise<std_msgs::String>("Actuator/Chassis",         1000);
  // Topics end
  roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::EscapeBehavior       escape_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::PatrolBehavior       patrol_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::GoalBehavior       goal_behavior(chassis_executor, blackboard);

  ros::Subscriber goalsub = n.subscribe("PyDecision/Goal", 1000, goalrccallback);
  ros::Subscriber twistsub = n.subscribe("PyDecision/Twist", 1000, twistrccallback);
  ros::Subscriber omegasub = n.subscribe("PyDecision/Omega", 1000, omegarccallback);
  auto command_thread= std::thread(Command);
  ros::Rate rate(10);
  // This structure itself can be consider as a decision tree styled Behavior Tree structure.
  while(ros::ok()){
    ros::spinOnce();
    enemy_pose.publish(blackboard->GetEnemy());
    self_pose.publish(blackboard->GetRobotMapPose());
    goal.publish(blackboard->GetGoal());
    std_msgs::String s;
    std::stringstream ss;
    if(blackboard->IsEnemyDetected()==true)
    {
      ss<<"true";
    }
    else
    {
      ss<<"false";
    }
    s.data = ss.str();
    is_detected.publish(s);
    std_msgs::String s2;
    std::stringstream ss2;
    if(blackboard->IsNewGoal()==true)
    {
      ss2<<"true";
    }
    else
    {
      ss2<<"false";
    }
    s2.data = ss2.str();
    is_newgoal.publish(s2);

    std_msgs::String s3;
    std::stringstream ss3;
    if(chassis_executor->Update()==roborts_decision::BehaviorState::RUNNING)
    {
      ss3<<"RUNNING";
    }
    else if(chassis_executor->Update()==roborts_decision::BehaviorState::FAILURE)
    {
      ss3<<"FAILURE";
    }
    else if(chassis_executor->Update()==roborts_decision::BehaviorState::SUCCESS)
    {
      ss3<<"SUCCESS";
    }
    else
    {
      ss3<<"IDLE"; // After cancel the task
    }
    
    s3.data = ss3.str();
    chassis_status.publish(s3);
    
    switch (command) {
      //back to boot area
      case '1':
        enable_flag = false;
        back_boot_area_behavior.Run();
        break;
        //patrol
      case '2':
        enable_flag = false;
        patrol_behavior.Run();
        break;
        //chase.
      case '3':
        enable_flag = false;
        chase_behavior.Run();
        break;
        //search
      case '4':
        enable_flag = false;
        search_behavior.Run();
        break;
        //escape.
      case '5':
        enable_flag = false;
        escape_behavior.Run();
        break;
        //goal.
      case '6':
        enable_flag = false;
        goal_behavior.Run();
        break;
      case '7':
        enable_flag =true;
        break;
      case 27:
        enable_flag = false;
        if (command_thread.joinable()){
          command_thread.join();
        }
        return 0;
      default:
        enable_flag = false;
        break;
    }
    rate.sleep();
  }


  return 0;
}

void Command() {

  while (command != 27) {
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "*********************************please send a command********************************" << std::endl;
    std::cout << "1: back boot area behavior" << std::endl
              << "2: patrol behavior" << std::endl
              << "3: chase_behavior" << std::endl
              << "4: search behavior" << std::endl
              << "5: escape behavior" << std::endl
              << "6: goal behavior" << std::endl
              << "7: Listen to Python"<<std::endl
              << "esc: exit program" << std::endl;
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6' && command != '7' && command != 27) {
      std::cout << "please input again!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

  }
}

void goalrccallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  nextGoal.header.frame_id = "map";
  nextGoal.pose.position.x = msg->position.x;
  nextGoal.pose.position.y = msg->position.y;
  nextGoal.pose.position.z = msg->position.z;
  
  nextGoal.pose.orientation.x = msg->orientation.x;
  nextGoal.pose.orientation.y = msg->orientation.y;
  nextGoal.pose.orientation.z = msg->orientation.z;
  nextGoal.pose.orientation.w = msg->orientation.w;
  if(enable_flag==true)
  {
    chassis_executor->Execute(nextGoal);
  }
}
void twistrccallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  twist.linear.x = msg->linear.x;
  twist.linear.y = msg->linear.y;
  twist.linear.z = msg->linear.z;
  
  twist.angular.x = msg->angular.x;
  twist.angular.y = msg->angular.y;
  twist.angular.z = msg->angular.z;
  if(enable_flag==true)
  {
    chassis_executor->Execute(twist);
  }
}
void omegarccallback(const roborts_msgs::TwistAccel::ConstPtr& msg)
{
  omega.twist.linear.x = msg->twist.linear.x;
  omega.twist.linear.y = msg->twist.linear.y;
  omega.twist.linear.z = msg->twist.linear.z;

  omega.twist.angular.x= msg->twist.angular.x;
  omega.twist.angular.y= msg->twist.angular.y;
  omega.twist.angular.z= msg->twist.angular.z;

  omega.accel.linear.x = msg->accel.linear.x;
  omega.accel.linear.y = msg->accel.linear.y;
  omega.accel.linear.z = msg->accel.linear.z;

  omega.accel.angular.x= msg->accel.angular.x;
  omega.accel.angular.y= msg->accel.angular.y;
  omega.accel.angular.z= msg->accel.angular.z;
  if(enable_flag==true)
  {
    chassis_executor->Execute(omega);
  }
}


// May be I will create some callbacks for receiving msg from python

