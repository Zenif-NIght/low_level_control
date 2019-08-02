#include "turtlebot_sim/line_world_node.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/LaserScan.h"
#include <limits>
#include <math.h>
#include <tf/tf.h>

#include <sstream>

#define PI 3.14

using namespace turtlebot_sim;

Line::Line( const Eigen::Vector2d & q1, const Eigen::Vector2d & q2):
    q1(q1),
    q2(q2)
{
    angle = LineWorld::calculateAngle(q1, q2);
}

Line::Line(const Line & old):
    q1(old.q1),
    q2(old.q2),
    angle(old.angle)
{}

LineWorld::LineWorld(const std::string & vehicle_namespace, const std::string & inertial_frame,
                     const std::string & robot_frame, double loop_rate, int n_lines,
                     double max_dist, std::vector<turtlebot_sim::Line> lines) :
    frame_id(inertial_frame),
    robot_frame(robot_frame),
    vehicle_namespace(vehicle_namespace),
    lines(lines),
    n_lines(n_lines),
    d_max(max_dist),
    loop_rate(loop_rate)
{
    ///TODO: Read these values in from the parameter server
    Eigen::Vector2d q1, q2;

    // Advertize visualization messages
    line_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("line_world_lines", 10);
    laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
    createWorldVisualizationMessage();

    // Subscribe to odometry
    sub_odom = nh.subscribe("odom", 1, &LineWorld::odomCallback, this);

    // Initialize angles and intersection points
    angles = Eigen::VectorXd::LinSpaced(n_lines+1,0.0,2*PI);
    intersection_points = Eigen::MatrixXd::Zero(2, n_lines);
    for(int k = 0; k < n_lines; k++) {
        // Create rotation matrix
        Eigen::Matrix2d R;
        double c = std::cos(angles(k));
        double s = std::sin(angles(k));
        R << c, -s, s, c;

        // Rotate unit vector
        Eigen::Vector2d unit; unit << 1, 0;
        Eigen::Vector2d result = R * unit;
        intersection_points.col(k) = result;
    }
}

LineWorld::~LineWorld() {

}

double LineWorld::calculateAngle(const Eigen::Vector2d & q1, const Eigen::Vector2d & q2){
    Eigen::Vector2d diff = q2 - q1;
    return atan2(diff(1), diff(0));
}

double LineWorld::adjustAngle(double angle) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    return std::atan2(s, c);
}

void LineWorld::createWorldVisualizationMessage(){
    // ********* Visualize the lines ******** //
    // Create a visualization marker
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = "line_world";
    marker.type = marker.CYLINDER;
    marker.action = marker.ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.25;
    marker.color.b = marker.color.g = 0.0;
    marker.color.r = 1.0;
    marker.color.a = 1;
    marker.lifetime = ros::Duration(0);
    marker.frame_locked = true;

    // Initialize the marker pose
    marker.pose.position.z = -.25;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);


    // ********* Visualize the Lines ******** //
    // Initialize the marker for drawing lines
    marker.scale.x = 0.1;
    marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.0;
    marker.type = marker.LINE_STRIP;


    // Loop through all points
    int marker_id = 0;
    world_markers.markers.clear();
    for(std::vector<Line>::iterator iter = lines.begin(); iter != lines.end(); iter++, marker_id++) {
        // Create the first point
        geometry_msgs::Point pnt1, pnt2;
        pnt1.x = iter->q1(0);
        pnt1.y = iter->q1(1);
        pnt1.z = -.25;

        // Create the second point
        pnt2.x = iter->q2(0);
        pnt2.y = iter->q2(1);
        pnt2.z = -.25;

        // Add Points to the marker
        marker.points.clear();
        marker.points.push_back(pnt1);
        marker.points.push_back(pnt2);
        marker.colors.push_back(marker.color);
        marker.colors.push_back(marker.color);

        // Add the marker
        marker.id = marker_id;
        world_markers.markers.push_back(marker);
    }
}

void LineWorld::publishWorldVisualization() {
    line_vis_pub.publish(world_markers);
}

void LineWorld::odomCallback(const nav_msgs::Odometry::ConstPtr & msg){
    odom = msg;
}

bool LineWorld::get2DTransformedPose(geometry_msgs::Pose2D & pose){
    if(!odom)
        return false;

    // Initialize output 3D pose
    geometry_msgs::PoseStamped pose3d;
    pose3d.header.frame_id = frame_id;
    pose3d.header.stamp = odom->header.stamp;

    // Initialize odom pose
    geometry_msgs::PoseStamped pose_odom;
    pose_odom.header.frame_id = odom->header.frame_id;
    pose_odom.header.stamp = odom->header.stamp;
    pose_odom.pose.orientation = odom->pose.pose.orientation;
    pose_odom.pose.position = odom->pose.pose.position;

    // Tranform odom to the correct frame
    try {
        tf_listener.waitForTransform(frame_id,odom->header.frame_id, odom->header.stamp,ros::Duration(0.5));
        tf_listener.transformPose(frame_id, pose_odom, pose3d);
    }
    catch(tf::TransformException ex) {
        ROS_ERROR("LineWorld::get2DTransformedPose() %s", ex.what());
        return false;
    }

    // Extract 3D pose
    pose.theta = tf::getYaw(pose3d.pose.orientation);
    pose.x = pose3d.pose.position.x;
    pose.y = pose3d.pose.position.y;
    return true;
}

bool LineWorld::updateIntersectionPoints() {
    // Get the current position
    if(!get2DTransformedPose(latest_pose)) {
        return false;
    }
    Eigen::Vector2d position; position<< latest_pose.x, latest_pose.y;
    double orientation = latest_pose.theta;

    /// TODO: calculate intersections with lines
    Eigen::Vector2d max_vec; max_vec << d_max, 0;
    distances.clear();
    for(int k = 0; k < n_lines; k++) {
        // Create rotation matrix
        Eigen::Matrix2d R;
        double c = std::cos(angles(k) + orientation);
        double s = std::sin(angles(k) + orientation);
        R << c, -s, s, c;

        // Rotate the vector
        Eigen::Vector2d result = R * max_vec;
        result(0) += latest_pose.x;
        result(1) += latest_pose.y;

        // Limit the rotated max vector by obstacle intersection
        distances.push_back(static_cast<float>(calculateMaxPosition(position, result)));

        // Set the intersection point
        intersection_points.col(k) = result;
    }
    return true;
}

bool LineWorld::publishIntersectionPoints() {
    if (!updateIntersectionPoints()) {
        return false;
    }

    // ********* Visualize the lines ******** //
    // Create a visualization marker
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = "line_intersections";
    marker.type = marker.CYLINDER;
    marker.action = marker.ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.25;
    marker.color.b = marker.color.g = 0.0;
    marker.color.r = 1.0;
    marker.color.a = 1;
    marker.lifetime = ros::Duration(0);
    marker.frame_locked = true;

    // Initialize the marker pose
    marker.pose.position.z = -.25;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);


    // ********* Visualize the Lines ******** //
    // Initialize the marker for drawing lines
    marker.scale.x = 0.1;
    marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.0;
    marker.type = marker.LINE_STRIP;

    // Set initial point as the robot point
    geometry_msgs::Point pnt1;
    pnt1.x = latest_pose.x;
    pnt1.y = latest_pose.y;
    pnt1.z = -.25;

    // Loop through all points
    int marker_id = 0;
    sensor_markers.markers.clear();

    for(int k = 0; k < n_lines; k++) {
        // Create the second point
        geometry_msgs::Point pnt2;
        pnt2.x = intersection_points(0, k);
        pnt2.y = intersection_points(1, k);
        pnt2.z = -.25;

        // Add Points to the marker
        marker.points.clear();
        marker.points.push_back(pnt1);
        marker.points.push_back(pnt2);
        marker.colors.push_back(marker.color);
        marker.colors.push_back(marker.color);

        // Add the marker
        marker.id = marker_id++;
        sensor_markers.markers.push_back(marker);
    }

    line_vis_pub.publish(sensor_markers);
    return true;
}

void LineWorld::publishLaserScan() {
    // Create laser scan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = odom->header.stamp;
    scan.header.frame_id = vehicle_namespace +  "/" + robot_frame;
    scan.ranges = distances;
    scan.angle_max = static_cast<float>(angles(n_lines-1));
    scan.angle_min = static_cast<float>(angles(0));
    scan.range_max = static_cast<float>(d_max);
    scan.range_min = 0.0;
    scan.scan_time = static_cast<float>(loop_rate);
    scan.time_increment = 0.0;
    scan.angle_increment = static_cast<float>(angles(1) - angles(0));

    laser_pub.publish(scan);

}

void LineWorld::publishVisualization() {
    publishWorldVisualization();
    if (publishIntersectionPoints()) {
        publishLaserScan();
    }
}

bool LineWorld::get_line_intersection(double p0_x, double p0_y, double p1_x, double p1_y,
    double p2_x, double p2_y, double p3_x, double p3_y, double & i_x, double &i_y)
{
    double s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
    s10_x = p1_x - p0_x;
    s10_y = p1_y - p0_y;
    s32_x = p3_x - p2_x;
    s32_y = p3_y - p2_y;

    denom = s10_x * s32_y - s32_x * s10_y;
    if (std::abs(denom) < .0001)
        return false; // Collinear
    bool denomPositive = denom > 0;

    s02_x = p0_x - p2_x;
    s02_y = p0_y - p2_y;
    s_numer = s10_x * s02_y - s10_y * s02_x;
    if ((s_numer < 0) == denomPositive)
        return false; // No collision

    t_numer = s32_x * s02_y - s32_y * s02_x;
    if ((t_numer < 0) == denomPositive)
        return false; // No collision

    if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
        return false; // No collision
    // Collision detected
    t = t_numer / denom;
    i_x = p0_x + (t * s10_x);
    i_y = p0_y + (t * s10_y);

    return true;
}

double LineWorld::calculateMaxPosition(const Eigen::Vector2d & q0, Eigen::Vector2d & q1) {

    // Extract first line segment variables
    double p0_x, p0_y, p1_x, p1_y;
    p0_x = q0(0); p0_y = q0(1);
    p1_x = q1(0); p1_y = q1(1);

    // Loop through all obstacle line segments
    double dist_min = d_max;
    for(std::vector<Line>::iterator iter = lines.begin(); iter != lines.end(); iter++) {
        // Extract second line segment variables
        double p2_x, p2_y, p3_x, p3_y;
        p2_x = iter->q1(0); p2_y = iter->q1(1);
        p3_x = iter->q2(0); p3_y = iter->q2(1);

        // Determine if there is an intersection
        double i_x = 0.0;
        double i_y = 0.0;
        if (get_line_intersection(p0_x, p0_y, p1_x, p1_y,
                                  p2_x, p2_y, p3_x, p3_y,
                                  i_x, i_y)) {
            // Check to see if the new point is closer
            Eigen::Vector2d diff; diff << i_x - p0_x, i_y-p0_y;
            double nrm = diff.norm();
            if (nrm < dist_min) {
                q1 << i_x, i_y;
                dist_min = nrm;
            }
        }
    }
    return dist_min;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot_vehicle_node");

  // Initialize parameters to be read in
  std::string vehicle_namespace = "robot_1";
  std::string inertial_frame = "map";
  std::string robot_frame = "base_link";
  double loop_rate = 30;
  int n_lines = 10;
  double max_distance = 5;

  // Read in parameters
  ros::NodeHandle nh_loc("~");
  nh_loc.getParam("vehicle_namespace", vehicle_namespace);
  nh_loc.getParam("inertial_frame", inertial_frame);
  nh_loc.getParam("robot_frame", robot_frame);
  nh_loc.getParam("loop_rate", loop_rate);
  nh_loc.getParam("n_lines", n_lines);
  nh_loc.getParam("max_distance", max_distance);

  // Read in line parameters
  std::vector<double> x1, y1, x2, y2;
  nh_loc.getParam("x1", x1);
  nh_loc.getParam("y1", y1);
  nh_loc.getParam("x2", x2);
  nh_loc.getParam("y2", y2);
  ROS_ASSERT(x1.size() == y1.size() && y1.size() == x2.size() &&
             x2.size() == y2.size());

  // Create a vector of lines
  std::vector<turtlebot_sim::Line> lines;
  std::vector<double>::iterator x1_iter, x2_iter, y1_iter, y2_iter;
  x1_iter = x1.begin();
  y1_iter = y1.begin();
  x2_iter = x2.begin();
  y2_iter = y2.begin();
  for(; x1_iter != x1.end(); x1_iter++, y1_iter++, x2_iter++, y2_iter++) {
      Eigen::Vector2d q1, q2;
      q1 << *x1_iter, *y1_iter;
      q2 << *x2_iter, *y2_iter;
      lines.emplace_back(q1, q2);
  }


  LineWorld world(vehicle_namespace, inertial_frame,
                  robot_frame, loop_rate, n_lines,
                  max_distance, lines);

  ros::Rate lr(loop_rate);

  while (ros::ok())
  {
    world.publishVisualization();
    ros::spinOnce();
    lr.sleep();
  }

  return 0;
}
