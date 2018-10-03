#include "controllers/multi_agent_joystick.h"

#include <string>
#include <vector>

using namespace controllers;

MultiAgentJoystick::MultiAgentJoystick() {
    // Initialize class variables
    topic_base = "robot";
    backwards_select_pressed = false;
    forwards_select_pressed = false;
    selected_namespace = "";

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
        bool decrement = false;

        // Check backwards pressed
        if(msg->buttons.at(6) != 0) {
            if(!backwards_select_pressed) {
                backwards_select_pressed = true;
                decrement = true;                
            }
        } else {
            backwards_select_pressed = false;
        }

        // Check forward pressed
        bool increment = false;
        if(msg->buttons.at(7) != 0) {
            if(!forwards_select_pressed) {
                forwards_select_pressed = true;
                increment = true;
            }
        } else {
            forwards_select_pressed = false;
        }

        // Update the selected namespace
        if(decrement)
            changeNamespace(true);
        else if(increment)
            changeNamespace(false);

        // Output the new namespace
        if(decrement || increment) {
            std::string output = "New namespace = " + selected_namespace;            
        }
    }

}

/**
 * @brief MultiAgentJoystick::changeNamespace Updates the namespace topic
 * @param decrement: True implies to get the previous namespace in the set, false implies to get the next
 * @return true if namespace changed
 */
bool MultiAgentJoystick::changeNamespace(bool decrement){
    // Check to see if the namespace set has values
    if(namespace_set.size() == 0)
        return false;

    // Get an iterator to the current selected namespace
    std::set<std::string>::const_iterator it = namespace_set.find(selected_namespace);

    // Choose the first element if the namespace was not found or if there is only one element
    std::string new_namespace = "";
    if(it == namespace_set.end() || namespace_set.size() == 1) {
        new_namespace = *(namespace_set.begin());
        std::string output = "updating namespace to " + new_namespace;        
    }else if(decrement) { // Decrement just starts over for now
        new_namespace = *(namespace_set.begin());
    }else {
        // Increment the iterator
        it++;

        // Update the new namespace
        if(it == namespace_set.end()) // Check to see if the end is reached
            it = namespace_set.begin();
        new_namespace = *it;
    }

    // Update the namespace
    if(new_namespace.compare(selected_namespace) == 0)
        return false;
    else {
        selected_namespace = new_namespace;

        // Update the publisher
        std::string cmd_topic = "/" + selected_namespace + "cmd_vel";
        pub_command = n.advertise<geometry_msgs::Twist>(cmd_topic.c_str(), 1);

        // Output Robot being controlled
        std::string output = "\nMultiAgentJoystick - Controlling " + selected_namespace + "\n";
        return true;
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
