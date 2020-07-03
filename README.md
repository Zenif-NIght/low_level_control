# Project overview
This project has three key elements which are explained below:
1.  Dynamic models
2.  Controllers
3.  Turtlebot sim

For examples which use this project, see [Setup and Launch Repo](https://gitlab.com/droge-robotics/setup_and_launch/edit/master/README.md)

# Dynamic Models
This package has simple models for running the robot. An example is given in *unicycle.cpp*

Two important functions that should be modified for creating more complex dynamic models are:
1.  *Unicycle::updateOdometry(...)*: This performs Euler integration on the dynamics
2.  *Unicycle::commandVelocityCallback(...)*: This stores the commanded inputs to the system

# Controllers
This package contains controllers for controlling the robot. At the current time there are only two joystick controllers. You must install the joy package (for joystick drivers)

kinetic:

`sudo apt install ros-kinetic-ros-tutorials`

`sudo apt install ros-kinetic-joy`

melodic:

`sudo apt install ros-melodic-ros-tutorials`

`sudo apt install ros-melodic-joy`

See [http://wiki.ros.org/joy](http://wiki.ros.org/joy) to setup the joystick drivers correctly.

# Turtlebot sim
This package is used for the visualization of the robot. There are currently two nodes:
*  *simple_map_tf*: Produces a mapping from the odom frame to the map frame to allow for plotting of the vehicle in the map frame
*  *turtlebot_vehicle*: Produces the motion of the joint states for the plotting of the turtlebot

For adding additional visualizations, you may want to create another node like *turtlebot_vehicle* which publishes the correct joint positions and motions.

# Occupancy Grid
This package contains a occupancy grid implementation that can be generated off of configuration files at startup, or at run time via laser scan messages.
To launch a node that makes and maintains an occupancy grid run the following command:

    roslaunch occupancy_grid occupancy_grid.launch

The meaning of each argument in the occupancy_grid.launch launch file is listed below:

* loop_rate: The rate at which this node will publish messages, answer service calls, and respond to incoming messages
* world_file: This is the absolute path to a configuration file that defines what pixels of the occupancy grid will be occupied at startup
* resolution: How far apart each pixel of the occupancy grid will be
* service_topic: The topic that other nodes can use to receive the occupancy grid via a service call
* massage_topic: The topic that this node will publish the occupancy grid on
* tf_frame: The TF frame the grid should live in
* laser_scan_topic: The topic that this node will receive laser scan messages on so it can update the occupancy grid with new obstacles
* laser_scan_queue_length: How many laser scan messages this node will hold between updates
* laser_scan: A boolean flag that tells the launch file whether or not to have the node receive laser scan messages
* start_rviz: A boolean flag that tells the launch file whether or not it should spin-up RVIZ

The meaning of each part of the world configuration files is listed below;
* origin: The x, y, and yaw value of the 0,0 point on the occupancy grid in the global ROS frame
* height: The height of the occupancy grid
* width: The width of the occupancy grid
* line_width: The width of the lines added at startup and at run time
* line(): A incrementally increasing list of lines in the form, [x1, y1, x2, y2]

