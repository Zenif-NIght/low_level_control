#include "turtlebot_sim/turtlebot_vehicle.h"

using namespace turtlebot_sim;


Turtlebot_vehicle::Turtlebot_vehicle(){
    init();
}
Turtlebot_vehicle::~Turtlebot_vehicle(){

}
bool Turtlebot_vehicle::init(){
    // initialize ROS parameter

    std::string robot_model = nh_.param<std::string>("tb3_model", "");

    if (!robot_model.compare("burger"))
    {
      wheel_seperation_ = 0.160;
      turning_radius_   = 0.080;
      robot_radius_     = 0.105;
    }
    else if (!robot_model.compare("waffle") || !robot_model.compare("waffle_pi"))
    {
      wheel_seperation_ = 0.287;
      turning_radius_   = 0.1435;
      robot_radius_     = 0.220;
    }

    nh_.param("wheel_left_joint_name", joint_states_name_[LEFT],  std::string("wheel_left_joint"));
    nh_.param("wheel_right_joint_name", joint_states_name_[RIGHT],  std::string("wheel_right_joint"));
    nh_.param("joint_states_frame", joint_states_.header.frame_id, std::string("base_footprint"));

    // initialize variables
    wheel_speed_cmd_[LEFT]  = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
    last_position_[LEFT]    = 0.0;
    last_position_[RIGHT]   = 0.0;
    last_velocity_[LEFT]    = 0.0;
    last_velocity_[RIGHT]   = 0.0;

    joint_states_.name.push_back(joint_states_name_[LEFT]);
    joint_states_.name.push_back(joint_states_name_[RIGHT]);
    joint_states_.position.resize(2,0.0);
    joint_states_.velocity.resize(2,0.0);
    joint_states_.effort.resize(2,0.0);

    // initialize publishers
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);

    prev_update_time_ = ros::Time::now();
    return true;

}
bool Turtlebot_vehicle::update(){
    ros::Time time_now = ros::Time::now();
    ros::Duration step_time = time_now - prev_update_time_;
    prev_update_time_ = time_now;

    // joint_states
    updateJoint(step_time);
    joint_states_.header.stamp = time_now;
    joint_states_pub_.publish(joint_states_);

    return true;
}

void Turtlebot_vehicle::updateJoint(ros::Duration diff_time){

    double wheel_l, wheel_r; // rotation value of wheel [rad]
    double delta_s, delta_theta;
    double v[2], w[2];

    wheel_l = wheel_r     = 0.0;
    delta_s = delta_theta = 0.0;

    v[LEFT]  = wheel_speed_cmd_[LEFT];
    w[LEFT]  = v[LEFT] / WHEEL_RADIUS;  // w = v / r
    v[RIGHT] = wheel_speed_cmd_[RIGHT];
    w[RIGHT] = v[RIGHT] / WHEEL_RADIUS;

    last_velocity_[LEFT]  = w[LEFT];
    last_velocity_[RIGHT] = w[RIGHT];

    wheel_l = w[LEFT]  * diff_time.toSec();
    wheel_r = w[RIGHT] * diff_time.toSec();

    if(isnan(wheel_l))
    {
      wheel_l = 0.0;
    }

    if(isnan(wheel_r))
    {
      wheel_r = 0.0;
    }

    last_position_[LEFT]  += wheel_l;
    last_position_[RIGHT] += wheel_r;


    joint_states_.position[LEFT]  = last_position_[LEFT];
    joint_states_.position[RIGHT] = last_position_[RIGHT];
    joint_states_.velocity[LEFT]  = last_velocity_[LEFT];
    joint_states_.velocity[RIGHT] = last_velocity_[RIGHT];
}

void Turtlebot_vehicle::odometryCallback(const nav_msgs::OdometryConstPtr odom){
    double linear_velocity  = odom->twist.twist.linear.x;
    double angular_velocity = odom->twist.twist.angular.z;

    wheel_speed_cmd_[LEFT]  = linear_velocity - (angular_velocity * wheel_seperation_ / 2);
    wheel_speed_cmd_[RIGHT] = linear_velocity + (angular_velocity * wheel_seperation_ / 2);
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot_vehicle_node");
  Turtlebot_vehicle vehicle;

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    vehicle.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
