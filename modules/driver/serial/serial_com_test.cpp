#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <messages/EnemyPos.h>
#include <messages/ShootState.h>
#include <termios.h>
#include <fcntl.h>

//using namespace std;

geometry_msgs::Twist chassis_data;
messages::ShootState shoot_data;
float def_v = 0.3;

void init_data()
{
  chassis_data.linear.x  = 0.0;
  chassis_data.linear.y  = 0.0;
  chassis_data.linear.z  = 0.0;
  chassis_data.angular.z = 0.0;

  shoot_data.single_shoot = 0;
  shoot_data.continue_shoot = 0;
  shoot_data.run_friction_whell = 0;
  shoot_data.friction_whell_speed = 0;
}

void move(char cmd)
{
  init_data();
  switch(cmd)
  {
    case 'w':
      def_v = 0.3;
      chassis_data.linear.x  =  def_v;
      break;
    case 's':
      def_v = 0.3;
      chassis_data.linear.x  = -def_v;
      break;
    case 'a':
      def_v = 0.3;
      chassis_data.linear.y  =  def_v;
      break;
    case 'd':
      def_v = 0.3;
      chassis_data.linear.y  = -def_v;
      break;
    case 'r':
      def_v = 0.3;
      chassis_data.angular.z =  def_v;
      break;
    case 'f':
      def_v = 0.3;
      chassis_data.angular.z = -def_v;
      break;
    case 'u':
      def_v = 0;
      break;
    case 'h':
      shoot_data.continue_shoot = 0;
      shoot_data.run_friction_whell = 1;
      shoot_data.friction_whell_speed = 1300;
      break;
    case 'j':
      shoot_data.continue_shoot = 1;
      shoot_data.run_friction_whell = 1;
      shoot_data.friction_whell_speed = 1300;
      break;
  }
}

void kbhit()
{
  struct termios oldt, newt;
  char ch;
  int oldf;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  if(ch=='w'||ch=='s'||ch=='a'||ch=='d'||ch=='r'||ch=='f'||ch=='h'||ch=='j')
  {
    printf("You entered a character %c.", ch);
    printf("\nPlease enter your comment:\n");
    //ungetc(ch, stdin);
    move(ch);
  }
  //else
  //    ungetc(ch, stdin);
}

void ClearShootData() {
  shoot_data.single_shoot = 0;
  shoot_data.continue_shoot = 0;
  shoot_data.run_friction_whell = 1;
  shoot_data.friction_whell_speed = 1300;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "car_console");
  init_data();
  ros::NodeHandle n;
  ros::Publisher pub_chassis_cantrol = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher pub_shoot_control   = n.advertise<messages::ShootState>("shoot_cmd", 1000);
  ros::Rate loop_rate(10);
  printf("Please enter your comment:\n");
  while(n.ok())
  {
    ros::spinOnce();
    kbhit();
    pub_chassis_cantrol.publish(chassis_data);
    if (shoot_data.run_friction_whell != 0) {
      pub_shoot_control.publish(shoot_data);
      ClearShootData();
    }
    loop_rate.sleep();
  }
}
