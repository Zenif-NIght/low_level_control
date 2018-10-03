#include "controllers/multi_agent_joystick.h"

#include <string>
#include <vector>

using namespace controllers;

MultiAgentJoystick::MultiAgentJoystick() {
    // Initialize class variables
    topic_base = "robot";
    backwards_select_pressed = false;
    forwards_select_pressed = false;

    // Initialize the joystick command
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.linear.x = 0.0;

    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    // Advertize the command
    pub_command = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void MultiAgentJoystick::joystickCallback(const sensor_msgs::Joy_<std::allocator<void> >::ConstPtr &msg) {
    // Create a message from the first two elements of msg->axes
    if(msg->axes.size() < 2) {
        ROS_ERROR_THROTTLE(1.0, "MultiAgentJoystick::joystickCallback() Joystick command too small");
    } else {
        // Update the turtle command
        cmd.linear.x = msg->axes[1];
        cmd.angular.z = msg->axes[0];
    }

    // Process change of control
    if(msg->buttons.size() < 7) {
        ROS_ERROR_THROTTLE(1.0, "MultiAgentJoystick::joystickCallback() Insufficient joystick buttons");
    } else {
        // Check backwards pressed
        if(msg->buttons.at(6) != 0) {
            if(!backwards_select_pressed) {
                backwards_select_pressed = true;
                ROS_INFO("MultiAgentJoystick: Switching Robot - backwards");
            }
        } else {
            backwards_select_pressed = false;
        }

        // Check forward pressed
        if(msg->buttons.at(7) != 0) {
            if(!forwards_select_pressed) {
                forwards_select_pressed = true;
                ROS_INFO("MultiAgentJoystick: Switching Robot - forwards");
            }
        } else {
            forwards_select_pressed = false;
        }

    }

}

void MultiAgentJoystick::publishCommand(){
    pub_command.publish(cmd);
}

void MultiAgentJoystick::processTopics(){
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    std::string output = "\n";
    for(ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++)
    {
        const ros::master::TopicInfo& info = *it;
        std::string namespace_in = "";

        if( extractTopicNamespace(info.name, namespace_in) ) {
            namespace_set.insert(namespace_in);
            output += "topic: " + info.name + ", valid namespace = " + namespace_in + "\n";
        } else {
            output += "topic: " + info.name + "\n";
        }
    }
    //ROS_INFO_THROTTLE(1.0, output.c_str());

    // Output list of robots:
    output = "Valid robot namespaces found:";
    for(std::set<std::string>::iterator it = namespace_set.begin(); it != namespace_set.end(); it++) {
        output += *it + "\n";
    }
    ROS_INFO_THROTTLE(1.0, output.c_str());
}

bool MultiAgentJoystick::extractTopicNamespace(const std::string & topic_input, std::string & namespace_out )
{
    // Find first and second forward slashes
    size_t first_slash = topic_input.find('/');
    if(first_slash == std::string::npos) return false;
    size_t second_slash = topic_input.find('/', first_slash+1);
    if(second_slash == std::string::npos) return false;

    // Find the topic base
    size_t topic_base_pos = topic_input.find(topic_base, first_slash+1);

    // Check validity of the topic base position
    if(topic_base_pos == std::string::npos || topic_base_pos > second_slash)
        return false;

    // Extract the full namespace
    namespace_out = topic_input.substr(topic_base_pos, second_slash-topic_base_pos+1);
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "beginner_tutorials_joystick");

    // Create joystick instance
    MultiAgentJoystick tmp;

    // Subscribe to the topic
    ros::NodeHandle node;
    ros::Subscriber sub_joy = node.subscribe("/joy", 10, &MultiAgentJoystick::joystickCallback, &tmp);




    ros::Rate r(20);

    while(ros::ok()) {
        tmp.publishCommand();
        ros::spinOnce();
        r.sleep();

        tmp.processTopics();
    }

    return 0;
}
