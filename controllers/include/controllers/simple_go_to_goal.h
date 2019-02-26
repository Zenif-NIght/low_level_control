#ifndef __SIMPLE_GO_TO_GOAL__
#define __SIMPLE_GO_TO_GOAL__

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

    ros::NodeHandle n;
    ros::Publisher pub_command;

    geometry_msgs::Twist cmd;
    geometry_msgs::PoseStamped::ConstPtr goal;
    nav_msgs::Odometry::ConstPtr odom;

    tf::TransformListener tf_listener;


    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr & msg);

    void publishCommand();
    bool calculateCommand();

};

}
#endif // __SIMPLE_GO_TO_GOAL__
