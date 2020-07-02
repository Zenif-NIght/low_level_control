/**
 * @File: laser_scan_processor.cpp
 * @Date: June 2020
 * @Author: James Swedeen
 **/

/* Local Headers */
#include"occupancy_grid/laser_scan_processor.hpp"

/* ROS Headers */
#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/PointStamped.h>

/* TF Headers */
#include<tf/transform_listener.h>

/* C++ Headers */
#include<forward_list>
#include<string>
#include<array>
#include<cmath>
#include<functional>

LaserScanProcessor::LaserScanProcessor(const std::string& laser_scan_topic,
                                       const std::string& map_tf_id,
                                       const size_t       queue_length)
 : laser_scan_sub(this->c_nh.subscribe(laser_scan_topic, queue_length, &LaserScanProcessor::laserScanCallback, this)),
   map_tf_id(map_tf_id)
{}

std::forward_list<std::array<std::array<double,2>,2>> LaserScanProcessor::getObstacles()
{
  return std::move(this->obstacles);
}

void LaserScanProcessor::laserScanCallback(const sensor_msgs::LaserScan& msg)
{
  // Transform into lines
  bool last_was_obs = false;

  // Add an edge that connects any 2 adjacent visible obstacles
  for(size_t range_it = 0; range_it < msg.ranges.size(); range_it++)
  {
    // If there is a visible obstacle
    if((msg.ranges[range_it] >= msg.range_min) and
       (msg.ranges[range_it] <= msg.range_max))
    {
      if(last_was_obs)
      {
        std::array<std::array<double,2>,2> output;
        geometry_msgs::PointStamped        temp_point;
        float                              point_yaw;

        /* Second point */
        // Set time
        temp_point.header       = msg.header;
        temp_point.header.stamp = temp_point.header.stamp + ros::Duration(msg.time_increment * range_it);

        // Find angle
        point_yaw = msg.angle_min + (msg.angle_increment * range_it);

        // Find x and y
        temp_point.point.x = msg.ranges[range_it] * std::cos(point_yaw);
        temp_point.point.y = msg.ranges[range_it] * std::sin(point_yaw);
        temp_point.point.z = 0;

        // Transform to map frame
        try
        {
          this->transformer.transformPoint(this->map_tf_id, temp_point.header.stamp, temp_point, this->map_tf_id, temp_point);
        }
        catch(const tf::TransformException& ex)
        {
          ROS_WARN_STREAM(ex.what());
          continue;
        }

        // Store result
        output[1][0] = temp_point.point.x;
        output[1][1] = temp_point.point.y;

        /* First point */
        // Set time
        temp_point.header.stamp = temp_point.header.stamp - ros::Duration(msg.time_increment);

        // Find angle
        point_yaw = msg.angle_min - msg.angle_increment;

        // Find x and y
        temp_point.point.x = msg.ranges[range_it] * std::cos(point_yaw);
        temp_point.point.y = msg.ranges[range_it] * std::sin(point_yaw);
        temp_point.point.z = 0;

        // Transform to map frame
        try
        {
          this->transformer.transformPoint(this->map_tf_id, temp_point.header.stamp, temp_point, this->map_tf_id, temp_point);
        }
        catch(const tf::TransformException& ex)
        {
          ROS_WARN_STREAM(ex.what());
          continue;
        }

        // Store result
        output[0][0] = temp_point.point.x;
        output[0][1] = temp_point.point.y;

        this->obstacles.emplace_front(std::move(output));
      }

      last_was_obs = true;
    }
    else
    {
      last_was_obs = false;
    }
  }
}
/* laser_scan_processor.cpp */
