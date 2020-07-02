/**
 * @File: laser_scan_processor.hpp
 * @Date: June 2020
 * @Author: James Swedeen
 *
 * @brief
 * Used to process incoming laser scan information.
 **/

#ifndef OCCUPANCY_GRID_LASER_SCAN_PROCESSOR_HPP
#define OCCUPANCY_GRID_LASER_SCAN_PROCESSOR_HPP

/* ROS Headers */
#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<sensor_msgs/LaserScan.h>

/* TF Headers */
#include<tf/transform_listener.h>

/* C++ Headers */
#include<forward_list>
#include<string>
#include<array>

class LaserScanProcessor
{
public:
  /**
   * @Default Constructor
   **/
  LaserScanProcessor() = delete;
  /**
   * @Copy Constructor
   **/
  LaserScanProcessor(const LaserScanProcessor&) = delete;
  /**
   * @Move Constructor
   **/
  LaserScanProcessor(LaserScanProcessor&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the occupancy grid for use.
   *
   * @parameters
   * laser_scan_topic: The topic that laser scan messages come in on
   * map_tf_id: The TF frame of the global map
   * queue_length: How many laser scan messages should this object hold
   *               between calls of getObstacles
   **/
  LaserScanProcessor(const std::string& laser_scan_topic,
                     const std::string& map_tf_id,
                     const size_t       queue_length);
  /**
   * @Deconstructor
   **/
  ~LaserScanProcessor() noexcept = default;
  /**
   * @Assignment Operators
   **/
  LaserScanProcessor& operator=(const LaserScanProcessor&)  = delete;
  LaserScanProcessor& operator=(      LaserScanProcessor&&) = delete;
  /**
   * @getObstacles
   *
   * @brief
   * Returns the list of known obstacles that this object has collected and clears
   * the internal stash.
   **/
  std::forward_list<std::array<std::array<double,2>,2>> getObstacles();
private:
  /* Information inflow */
  ros::NodeHandle                                       c_nh;
  ros::Subscriber                                       laser_scan_sub;
  tf::TransformListener                                 transformer;
  std::string                                           map_tf_id;
  std::forward_list<std::array<std::array<double,2>,2>> obstacles;
  /**
   * @laserScanCallback
   **/
  void laserScanCallback(const sensor_msgs::LaserScan& msg);
};

#endif
/* laser_scan_processor.hpp */
