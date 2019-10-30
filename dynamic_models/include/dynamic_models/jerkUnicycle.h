/*******************************************************************************
* unicycle.h and unicycle.cpp hav been adapted from turtlebot3_fake.h and turtlebot3_fake.cpp
* See the ROBOTIS copyright below:
*
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef JERK_UNICYCLE_H_
#define JERK_UNICYCLE_H_

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace dynamic_models
{
class JerkUnicycle
{
public:
  JerkUnicycle();
  ~JerkUnicycle();
  bool init();
  bool update();

private:
  // ROS NodeHandle
  ros::NodeHandle nh_;

  // ROS Topic Publishers
  ros::Publisher odom_pub_;

  // ROS Topic Subscribers
  ros::Subscriber cmd_vel_sub_;

  // ROS time for updates
  ros::Time prev_update_time_;

  // State Variables
  double linear_velocity_;
  double angular_velocity_;
  double linear_acceleration_;
  double angular_acceleration_;
  double yaw_;
  nav_msgs::Odometry odom_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Control variables
  double linear_velocity_desired_;
  double angular_velocity_desired_;
  double k11_, k12_, k13_, k14_,  // Feedback matrix (note: a better representation would be
      k21_, k22_, k23_, k24_;     //   to use a linear algebra library such as Eigen3)

  // Function prototypes
  void commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg);
  bool updateOdometry(ros::Duration diff_time);
  void updateTF(geometry_msgs::TransformStamped& odom_tf);
};
}

#endif  // JERK_UNICYCLE_H_
