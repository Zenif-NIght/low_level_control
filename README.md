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




