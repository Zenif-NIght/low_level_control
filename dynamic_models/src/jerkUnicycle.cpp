#include "dynamic_models/jerkUnicycle.h"

using namespace dynamic_models;

JerkUnicycle::JerkUnicycle()
{
  init();
}

JerkUnicycle::~JerkUnicycle()
{
}

bool JerkUnicycle::init()
{
  // Initialize state variables
  linear_velocity_ = angular_velocity_ = linear_acceleration_ = angular_acceleration_ = 0.0;

  // Initialize desired velocities
  linear_velocity_desired_ = angular_velocity_desired_ = 0.0;

  // Initialize feedback matrix
  /*****
   * This matrix was calculated using matlab's LQR command. The system matrices used
   * were:
   *          [0 0 1 0]           [0 0]
   *          [0 0 0 1]           [0 0]
   *      A=  [0 0 0 0]        B= [1 0]
   *          [0 0 0 0]           [0 1]
   *
   * The cost matrices used where:
   *      Q = diag([10, 10, 0, 0]);
   *      R = diag([1, 1]);
   * *****/
  k11_ = 3.1623;  // Feedback used to calculate linear acceleration input
  k12_ = 0.0;
  k13_ = 2.5149;
  k14_ = 0.0;
  k21_ = 0.0;  // Feedback used to calculate angular acceleration input
  k22_ = 3.1623;
  k23_ = 0.0;
  k24_ = 2.5149;

  // Initialize the odometry position
  odom_.pose.pose.position.x = 0.0;
  odom_.pose.pose.position.y = 0.0;
  odom_.pose.pose.position.z = 0.0;
  yaw_ = 0.0;
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);

  // Intialize the odometry twist
  odom_.twist.twist.linear.x = linear_velocity_;
  odom_.twist.twist.linear.y = 0.0;
  odom_.twist.twist.linear.z = 0.0;
  odom_.twist.twist.angular.x = 0.0;
  odom_.twist.twist.angular.y = 0.0;
  odom_.twist.twist.angular.z = angular_velocity_;

  nh_.param("odom_frame", odom_.header.frame_id, std::string("odom"));
  nh_.param("base_frame", odom_.child_frame_id, std::string("base_footprint"));

  double pcov[36] = { 0.1, 0, 0, 0,   0, 0, 0, 0.1, 0, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
                      0,   0, 0, 1e6, 0, 0, 0, 0,   0, 0, 1e6, 0, 0, 0, 0,   0, 0, 0.2 };
  memcpy(&(odom_.pose.covariance), pcov, sizeof(double) * 36);
  memcpy(&(odom_.twist.covariance), pcov, sizeof(double) * 36);

  // initialize publishers
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 100);

  // initialize subscribers
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 100, &JerkUnicycle::commandVelocityCallback, this);

  prev_update_time_ = ros::Time::now();
  return true;
}

bool JerkUnicycle::update()
{
  ros::Time time_now = ros::Time::now();
  ros::Duration step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  // odom
  updateOdometry(step_time);
  odom_.header.stamp = time_now;
  odom_pub_.publish(odom_);

  // tf
  geometry_msgs::TransformStamped odom_tf;
  updateTF(odom_tf);
  tf_broadcaster_.sendTransform(odom_tf);

  return true;
}

void JerkUnicycle::commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg)
{
  linear_velocity_desired_ = cmd_vel_msg->linear.x;
  angular_velocity_desired_ = cmd_vel_msg->angular.z;
}

bool JerkUnicycle::updateOdometry(ros::Duration diff_time)
{
  double dt = diff_time.toSec();

  // Calculate control (Note: Using the linear algebra Eigen3 library would simplify notation)
  double u_a =
      -(k11_ * (linear_velocity_ - linear_velocity_desired_) + k12_ * (angular_velocity_ - angular_velocity_desired_) +
        k13_ * linear_acceleration_ + k14_ * angular_acceleration_);
  double u_alpha =
      -(k21_ * (linear_velocity_ - linear_velocity_desired_) + k22_ * (angular_velocity_ - angular_velocity_desired_) +
        k23_ * linear_acceleration_ + k24_ * angular_acceleration_);

  // Use Euler integration for updating the odometry
  odom_.pose.pose.position.x = odom_.pose.pose.position.x + dt * linear_velocity_ * std::cos(yaw_);
  odom_.pose.pose.position.y = odom_.pose.pose.position.y + dt * linear_velocity_ * std::sin(yaw_);
  yaw_ = yaw_ + dt * angular_velocity_;
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);

  // Use Euler integration to update velocity and acceleration states
  linear_velocity_ = linear_velocity_ + dt * linear_acceleration_;
  angular_velocity_ = angular_velocity_ + dt * angular_acceleration_;
  linear_acceleration_ = linear_acceleration_ + dt * u_a;
  angular_acceleration_ = angular_acceleration_ + dt * u_alpha;

  // Update the odometry twist
  odom_.twist.twist.linear.x = linear_velocity_;
  odom_.twist.twist.angular.z = angular_velocity_;

  return true;
}

void JerkUnicycle::updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom_.header;
  odom_tf.child_frame_id = odom_.child_frame_id;
  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "jerk_unicycle_dynamics_node");
  JerkUnicycle jerk_unicycle;

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    jerk_unicycle.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
