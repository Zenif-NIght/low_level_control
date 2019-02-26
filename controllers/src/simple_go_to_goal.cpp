#include "controllers/simple_go_to_goal.h"
#include "math.h"

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

    // ********** Transform data **************** //
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
    double v_des = v_nom*std::tanh(dist_to_goal);

    // Calculate orientation error

    // Calculate desired rotational velocity

    // Store command



    return true;
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
