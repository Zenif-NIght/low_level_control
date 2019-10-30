#include "controllers/joystick_control.h"

#include <string>
#include <vector>

using namespace controllers;

JoystickControl::JoystickControl()
{
  // Initialize the turtle command
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.linear.x = 0.0;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;

  // Advertize the command
  pub_command = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void JoystickControl::joystickCallback(const sensor_msgs::Joy_<std::allocator<void> >::ConstPtr& msg)
{
  // Create a message from the first two elements of msg->axes
  if (msg->axes.size() < 2)
  {
    ROS_ERROR_THROTTLE(1.0, "JoystickControl::joystickCallback() Joystick command too small");
  }
  else
  {
    // Update the turtle command
    cmd.linear.x = msg->axes[1];
    cmd.angular.z = msg->axes[0];
  }
}

void JoystickControl::publishCommand()
{
  pub_command.publish(cmd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "beginner_tutorials_joystick");

  // Create joystick instance
  JoystickControl tmp;

  // Subscribe to the topic
  ros::NodeHandle node;
  ros::Subscriber sub_joy = node.subscribe("/joy", 10, &JoystickControl::joystickCallback, &tmp);

  ros::Rate r(20);

  while (ros::ok())
  {
    tmp.publishCommand();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
