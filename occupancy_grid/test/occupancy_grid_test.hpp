/**
 * @file occupancy_grid_test.hpp
 * @author Ryan Jacobson (jacobsonryan28@gmail.com  )
 * @brief Contains the test fixture for the occupancy grid unit tests
 * @date 2020-05-27
 * 
 * @copyright Copyright (c) 2020
 */
#ifndef OCCUPANCY_GRID_TEST
#define OCCUPANCY_GRID_TEST
/* ROS Headers */
#include <ros/ros.h>

/* Library Headers */
#include <gtest/gtest.h>

/* STL Headers */
#include <string>
class OccupancyGridTest : public ::testing::Test
{
  public:
    OccupancyGridTest()
    : nh("~")
    {
    }
  protected:
    ros::NodeHandle nh;

};
#endif /* OCCUPANCY_GRID_TEST */
