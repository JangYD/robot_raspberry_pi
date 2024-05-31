#ifndef ROBOT_KEYBOARD_TELEOP_H
#define ROBOT_KEYBOARD_TELEOP_H

#define MAX_LIN_VEL 2.0
#define MAX_ANG_VEL 2.0

#define LIN_VEL_STEP_SIZE 0.2
#define ANG_VEL_STEP_SIZE 0.2

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <termios.h>

class robot_keyboard_teleop
{
public:
  robot_keyboard_teleop();
  int keyloop();

public:
  ros::NodeHandle nh_;
  ros::Publisher twist_pub;
  geometry_msgs::Twist twist;

  char key;

  double linear_vel;
  double angular_vel;

private:

};

#endif // ROBOT_KEYBOARD_TELEOP_H
