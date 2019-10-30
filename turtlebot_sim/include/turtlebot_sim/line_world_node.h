#ifndef LINE_WORLD_NODE_H_
#define LINE_WORLD_NODE_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

namespace turtlebot_sim
{
struct Line
{
  Line(const Eigen::Vector2d& q1, const Eigen::Vector2d& q2);
  Line(const Line& old);
  ~Line()
  {
  }

  // Members
  Eigen::Vector2d q1;
  Eigen::Vector2d q2;
  double angle;  // Angle between points 1 and 2
};

class LineWorld
{
public:
  LineWorld(const std::string& vehicle_namespace, const std::string& inertial_frame, const std::string& robot_frame,
            double loop_rate, int n_lines, double max_dist, std::vector<turtlebot_sim::Line> lines);
  ~LineWorld();

  static double calculateAngle(const Eigen::Vector2d& q1, const Eigen::Vector2d& q2);
  static double adjustAngle(double angle_in);
  static bool get_line_intersection(double p0_x, double p0_y, double p1_x, double p1_y, double p2_x, double p2_y,
                                    double p3_x, double p3_y, double& i_x, double& i_y);

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  void publishVisualization();

  const std::string frame_id;
  const std::string robot_frame;
  const std::string vehicle_namespace;

private:
  // ROS variables
  ros::NodeHandle nh;
  nav_msgs::Odometry::ConstPtr odom;
  ros::Publisher line_vis_pub;
  ros::Publisher laser_pub;
  tf::TransformListener tf_listener;
  ros::Subscriber sub_odom;

  // Line variables
  std::vector<turtlebot_sim::Line> lines;

  // Visualization parameters
  visualization_msgs::MarkerArray world_markers;
  visualization_msgs::MarkerArray sensor_markers;
  void createWorldVisualizationMessage();

  // Sensor variables
  int n_lines;  // Number of lines evenly spaced around vehicle
  Eigen::VectorXd angles;
  double d_max;
  double loop_rate;  // rate at which the sensors are updated
  std::vector<float> distances;
  Eigen::MatrixXd intersection_points;
  geometry_msgs::Pose2D latest_pose;

  // Transform functions
  bool get2DTransformedPose(geometry_msgs::Pose2D& pose);

  // Update sensors
  bool updateIntersectionPoints();
  double calculateMaxPosition(const Eigen::Vector2d& q1, Eigen::Vector2d& q2);

  // Publishing functions
  void publishWorldVisualization();
  bool publishIntersectionPoints();
  void publishLaserScan();
};
}  // end namespace turtlebot_sim

#endif  // LINE_WORLD_NODE_H_
