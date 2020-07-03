/**
 * @File: occupancy_grid_node.cpp
 * @Date: July 2020
 * @Author: James Swedeen
 *
 * @brief
 * Node that holds and updates a occupancy grid for other nodes to use.
 **/

/* Local Headers */
#include"occupancy_grid/occupancy_grid.hpp"
#include"occupancy_grid/laser_scan_processor.hpp"
#include"occupancy_grid/GetOccupancyGrid.h"

/* ROS Headers */
#include<ros/ros.h>
#include<std_msgs/Header.h>
#include<sensor_msgs/LaserScan.h>
#include<nav_msgs/OccupancyGrid.h>

/* C++ Headers */
#include<memory>
#include<mutex>
#include<functional>
#include<forward_list>
#include<array>

/**
 * @giveGrid
 *
 * @brief
 * Function used for the get grid service. Simply returns a copy of the grid.
 *
 * @parameters
 * res: The service request that will be filled with the grid information
 * occupancy_grid: The most up to date grid available
 * grid_mux: A mutex to protect the grid from race conditions
 * header: The header of the outbound message
 **/
bool giveGrid(occupancy_grid::GetOccupancyGridRequest&,
              occupancy_grid::GetOccupancyGridResponse& res,
              const OccupancyGrid&                      occupancy_grid,
              std::mutex&                               grid_mux,
              std_msgs::Header&                         header) noexcept;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "occupancy_grid");
  ros::NodeHandle m_nh;
  ros::NodeHandle p_nh("~");

  /* Stack variables */
  std::shared_ptr<OccupancyGrid>      grid;
  std::mutex                          grid_mux;
  std::unique_ptr<LaserScanProcessor> laser_scan;

  ros::Publisher     grid_pub;
  ros::ServiceServer grid_srv;

  std::string      service_topic;
  std::string      message_topic;
  std_msgs::Header header;

  std::string laser_scan_topic;
  int         laser_scan_queue_length(0);

  /* Initialize variables */
  header.seq   = 0;
  header.stamp = ros::Time::now();

  // Get topic parameters
  if(!p_nh.getParam("service_topic",    service_topic)    or
     !p_nh.getParam("message_topic",    message_topic)    or
     !p_nh.getParam("tf_frame",         header.frame_id))
  {
    ROS_ERROR("Couldn't find topic information");
  }

  if(p_nh.getParam("laser_scan_topic",        laser_scan_topic) and
     p_nh.getParam("laser_scan_queue_length", laser_scan_queue_length))
  {
    laser_scan.reset(new LaserScanProcessor(laser_scan_topic, header.frame_id, laser_scan_queue_length));
  }

  grid = OccupancyGrid::makeOccupancyGrid(p_nh);

  grid_pub = m_nh.advertise<nav_msgs::OccupancyGrid>(message_topic, 2);
  grid_srv = m_nh.advertiseService(service_topic,
                                   boost::function<bool(occupancy_grid::GetOccupancyGridRequest&,occupancy_grid::GetOccupancyGridResponse&)>(
                                     std::bind(giveGrid,
                                               std::placeholders::_1,
                                               std::placeholders::_2,
                                               std::ref(*grid),
                                               std::ref(grid_mux),
                                               std::ref(header))));
  /* Run node */
  ros::Rate loop_rate(p_nh.param("loop_rate", 30));

  while(m_nh.ok())
  {
    ros::spinOnce();
    grid_mux.lock();

    // Update grid
    header.stamp = ros::Time::now();
    header.seq++;
    grid->setLoadTime(header.stamp);

    if(nullptr != laser_scan.get())
    {
      std::forward_list<std::array<std::array<double,2>,2>> obstacles = laser_scan->getObstacles();

      for(auto obs_it = obstacles.cbegin(); obs_it != obstacles.cend(); obs_it++)
      {
        grid->setLine(*obs_it, true);
      }
    }

    // Publish grid
    {
      nav_msgs::OccupancyGrid grid_msg;

      grid_msg.header = header;
      grid_msg.info   = grid->cgetInfo();
      grid_msg.data   = grid->cgetOccupancyGrid().reshape(0, 1);

      grid_pub.publish(grid_msg);
    }

    grid_mux.unlock();
    loop_rate.sleep();
  }

  exit(EXIT_SUCCESS);
}


bool giveGrid(occupancy_grid::GetOccupancyGridRequest&,
              occupancy_grid::GetOccupancyGridResponse& res,
              const OccupancyGrid&                      occupancy_grid,
              std::mutex&                               grid_mux,
              std_msgs::Header&                         header) noexcept
{
  std::lock_guard<std::mutex> lock(grid_mux);

  res.grid.header = header;
  res.grid.info   = occupancy_grid.cgetInfo();
  res.grid.data   = occupancy_grid.cgetOccupancyGrid().reshape(0, 1);

  return true;
}

/* occupancy_grid_node.cpp */
