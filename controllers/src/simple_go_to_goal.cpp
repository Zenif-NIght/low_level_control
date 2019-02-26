#include "controllers/simple_go_to_goal.h"
#include "math.h"

using namespace controllers;

SimpleGoToGoal::SimpleGoToGoal(): v_nom(1.0), k_w(1.0), k_v(0.5) {
    // Read in constants
    ros::NodeHandle nh("~");
    nh.getParam("nominal_velocity", v_nom);
    nh.getParam("trans_vel_gain", k_v);
    nh.getParam("rot_vel_gain", k_w);


    // Initialize the velocity command
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.linear.x = 0.0;

    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    // Advertize the command
    pub_command = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscribe to goal and odometry
    sub_goal = n.subscribe("goal", 1, &SimpleGoToGoal::goalCallback, this);
    sub_odom = n.subscribe("odom", 1, &SimpleGoToGoal::odomCallback, this);
}

void SimpleGoToGoal::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg){
    goal = msg;
}

void SimpleGoToGoal::odomCallback(const nav_msgs::Odometry::ConstPtr & msg){
    odom = msg;
}

void SimpleGoToGoal::publishCommand()
{
    pub_command.publish(cmd);
}

bool SimpleGoToGoal::calculateCommand(){
    // ********** Transform data **************** //
    // Check to see if data has arrived
    bool execute = true;
    if(!goal) {
        ROS_WARN_THROTTLE(5.0, "SimpleGoToGoal::calculateCommand() goal not yet received");
        execute = false;
    }
    if(!odom) {
        ROS_WARN_THROTTLE(5.0, "SimpleGoToGoal::calculateCommand() odometry not yet received");
        execute = false;
    }
    if(!execute) {
        return false;
    }

    // Store latest data
    geometry_msgs::PoseStamped goal_latest; // = *goal;
    nav_msgs::Odometry odom_latest = *odom;

    // Transform goal to odom frame
    try {
        tf_listener.transformPose(odom_latest.header.frame_id, *goal, goal_latest);
    }
    catch(tf::TransformException ex) {
        ROS_ERROR("SimpleGoToGoal::calculateCommand() %s", ex.what());
        return false;
    }

    // ********** Calculate command **************** //
    // Calculate desired translational velocity
    double del_x, del_y;        // Calculate distance to the goal
    del_x = goal_latest.pose.position.x - odom_latest.pose.pose.position.x;
    del_y = goal_latest.pose.position.y - odom_latest.pose.pose.position.y;
    double dist_to_goal = std::sqrt(del_x*del_x + del_y*del_y); // norm of the difference vector
    double v_des = k_v * dist_to_goal;
    v_des = std::min(v_des, v_nom);

    // Calculate orientation error
    double yaw = tf::getYaw(odom_latest.pose.pose.orientation);
    double yaw_des = std::atan2(del_y, del_x);
    double yaw_err = std::atan2(std::sin(yaw_des-yaw), std::cos(yaw_des - yaw));

    // Calculate desired rotational velocity
    double w_des = k_w*yaw_err;

    // Store command
    cmd.linear.x = v_des;
    cmd.angular.z = w_des;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_go_to_goal");
    //ros::NodeHandle node;
    ros::start();

    ros::Rate r(20);
    SimpleGoToGoal ctrl;

    while(ros::ok()) {
        ctrl.calculateCommand();
        ctrl.publishCommand();
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
