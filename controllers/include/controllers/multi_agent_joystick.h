#ifndef MULTI_AGENT_JOYSTICK_H
#define MULTI_AGENT_JOYSTICK_H

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include "geometry_msgs/Twist.h"

#include <set>

namespace controllers
{
class MultiAgentJoystick
{
public:
  MultiAgentJoystick();

public:
  ros::NodeHandle n;
  ros::Publisher pub_command;

  void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg);

  void publishCommand();

  void processTopics();

private:
  std::string topic_base;
  bool backwards_select_pressed;
  bool forwards_select_pressed;
  std::string selected_namespace;
  std::set<std::string> namespace_set;

  geometry_msgs::Twist cmd;

  bool extractTopicNamespace(const std::string& topic_input, std::string& namespace_out);
  bool changeNamespace(bool decrement);
};
}
#endif
