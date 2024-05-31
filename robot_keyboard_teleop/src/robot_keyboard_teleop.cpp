#include "robot_keyboard_teleop.h"

static const char* msg = R"(

    Moving Command :
          w
    a     s     d

    CTRL-C quit
                  )";

robot_keyboard_teleop::robot_keyboard_teleop()
{

}

int robot_keyboard_teleop::keyloop()
{
  int ch;
  struct termios oldt;
  struct termios newt;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv, "robot_keyboard_teleop");
  ros::NodeHandle nh;
  robot_keyboard_teleop key_teleop;
  key_teleop.twist_pub = key_teleop.nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  geometry_msgs::Twist twist;

  ROS_INFO("%s" , msg);

  while(ros::ok())
  {
    key_teleop.key = key_teleop.keyloop();

    if(key_teleop.key == 'w')
    {
      key_teleop.linear_vel += LIN_VEL_STEP_SIZE;
      ROS_INFO_STREAM("linear : " << key_teleop.linear_vel << " angular : " <<  key_teleop.angular_vel  << " key : " << key_teleop.key);
    }
    else if(key_teleop.key == 's')
    {
      key_teleop.linear_vel -= LIN_VEL_STEP_SIZE;
      ROS_INFO_STREAM("linear : " << key_teleop.linear_vel << " angular : " <<  key_teleop.angular_vel  << " key : " << key_teleop.key);
    }
    else if(key_teleop.key == 'a')
    {
      key_teleop.angular_vel += ANG_VEL_STEP_SIZE;
      ROS_INFO_STREAM("linear : " << key_teleop.linear_vel << " angular : " <<  key_teleop.angular_vel  << " key : " << key_teleop.key);
    }
    else if(key_teleop.key == 'd')
    {
      key_teleop.angular_vel -= ANG_VEL_STEP_SIZE;
      ROS_INFO_STREAM("linear : " << key_teleop.linear_vel << " angular : " <<  key_teleop.angular_vel  << " key : " << key_teleop.key);
    }
    else
    {
      key_teleop.linear_vel = 0.0;
      key_teleop.angular_vel = 0.0;
    }

    if(key_teleop.key == '\x03')
    {
      break;
    }

    twist.linear.x = key_teleop.linear_vel;
    twist.angular.z = key_teleop.angular_vel;
    key_teleop.twist_pub.publish(twist);
    ros::spinOnce();
  }
  return 0;
}

