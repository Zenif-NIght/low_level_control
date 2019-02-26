#include "controllers/simple_go_to_goal.h"

using namespace controllers;

SimpleGoToGoal::SimpleGoToGoal() {

}

void SimpleGoToGoal::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg){
    goal = msg;
}

void SimpleGoToGoal::odomCallback(const nav_msgs::Odometry::ConstPtr & msg){
    odom = msg;
}

void SimpleGoToGoal::publishCommand()
{

}

bool SimpleGoToGoal::calculateCommand(){

    // Check to see if data has arrived
    bool execute = true;
    if(!goal) {
        ROS_WARN_THROTTLE(1.0, "SimpleGoToGoal::calculateCommand() goal not yet received");
        execute = false;
    }
    if(!odom) {
        ROS_WARN_THROTTLE(1.0, "SimpleGoToGoal::calculateCommand() odometry not yet received");
        execute = false;
    }
    if(!execute)
        return false;

    // Store latest data
    geometry_msgs::PoseStamped goal_latest = *goal;
    nav_msgs::Odometry odom_latest = *odom;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_go_to_goal");

    ros::Rate r(20);

    while(ros::ok()) {
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
