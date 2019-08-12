#ifndef SIMPLE_GO_TO_GOAL_H
#define SIMPLE_GO_TO_GOAL_H

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace controllers
{
class SimpleGoToGoal
{
 public:
  SimpleGoToGoal();

 public:
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

  void publishCommand();
  bool calculateCommand();

 private:
  // ROS variables
  ros::NodeHandle n;
  ros::Publisher pub_command;
  ros::Publisher pub_path;
  ros::Subscriber sub_goal;
  ros::Subscriber sub_odom;
  tf::TransformListener tf_listener;

  // Message storage variables
  geometry_msgs::Twist cmd;
  geometry_msgs::PoseStamped goal;
  nav_msgs::Odometry::ConstPtr odom;

  // Flags
  bool goal_received;  // True if a goal location has been received, false otherwise

  // Nominal command values
  double v_nom;  // Nominal translational velocity
  double k_w;    // Gain on rotational velocity
  double k_v;    // Gain on translational velocity
};
}
#endif  // SIMPLE_GO_TO_GOAL_H
