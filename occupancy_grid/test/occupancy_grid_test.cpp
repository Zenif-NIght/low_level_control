/**
 * @file occupancy_grid_test.cpp
 * @author Ryan Jacobson (jacobsonryan28@gmail.com)
 * @brief Unit tests for the OccupancyGrid class
 * @date 2020-05-19
 *
 * @copyright Copyright (c) 2020
 */

/**
 * @Updates by: James Swedeen
 * @Date: July 2020
 **/

/* Local Headers */
#include "occupancy_grid/occupancy_grid.hpp"
#include "occupancy_grid_test.hpp"

/* Libraries */
#include <gtest/gtest.h>

/* ROS Headers */
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

/* STL Headers */
#include<vector>
#include<array>
#include<string>
#include<memory>

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "occupancy_grid_test");
  return RUN_ALL_TESTS();
}

TEST_F(OccupancyGridTest, IsOccupiedUnOccupied)
{
  geometry_msgs::Pose  origin;
  std::array<double,2> point;
  origin.position.x = -25.0;
  origin.position.y = -25.0;
  origin.position.z = 0.0;
  OccupancyGrid grid(1, 0.1, origin, 50, 50);

  point[0] = 0.0;
  point[1] = 0.0;
  for (int i = 0; i < 50; i++)
  {
    for (int j = 0; j < 50; j++)
    {
      EXPECT_FALSE(grid.isOccupied(point));
      point[1] += .1;
    }
    point[0] += .1;
  }
}

TEST_F(OccupancyGridTest, IsOccupiedOccupied)
{
  std::vector<std::array<std::array<double,2>,2>> lines;
  std::array<std::array<double,2>,2>              line;
  geometry_msgs::Pose  origin;
  std::array<double,2> point;
  origin.position.x = 0;
  origin.position.y = 0;
  origin.position.z = 0.0;
  line[0][0]        = 0.0;
  line[0][1]        = 0.0;
  line[1][0]        = 50.0;
  line[1][1]        = 0.0;
  lines.push_back(line);
  point[0] = 0;
  point[1] = 0;
  OccupancyGrid grid(lines, 3, 1, origin, 50, 50);
  for (int i = 0; i < 50; i++)
    {
      EXPECT_TRUE(grid.isOccupied(point));
      point[1] += 1;
  }
}

TEST_F(OccupancyGridTest, SetPoint)
{
  geometry_msgs::Pose  origin;
  std::array<double,2> point;
  origin.position.x = -25.0;
  origin.position.y = -25.0;
  origin.position.z = 0.0;

  point[0] = -25.0;
  point[1] = 1.0;
  OccupancyGrid grid(1, 0.1, origin, 50, 50);

  grid.setPoint(point, true);
  EXPECT_TRUE(grid.isOccupied(point));
}

TEST_F(OccupancyGridTest, MakeOccupancyGridAllParams)
{
  double resolution = 1;
  double width      = 50;
  double height     = 50;
  double line_width = 1;
  std::vector<double> origin, line;
  origin.push_back(1.0);
  origin.push_back(1.0);
  origin.push_back(0.0);
  line.push_back(10.0);
  line.push_back(0.0);
  line.push_back(20.0);
  line.push_back(0.0);

  nh.setParam("resolution", resolution);
  nh.setParam("origin", origin);
  nh.setParam("width", width);
  nh.setParam("height", height);
  nh.setParam("line_width", line_width);
  nh.setParam("line0", line);

  std::shared_ptr<OccupancyGrid> grid = OccupancyGrid::makeOccupancyGrid(this->nh);

  std::array<double,2> point;
  point[1] = 21.0;
  point[0] = 1.0;

  EXPECT_TRUE(grid->isOccupied(point));
  EXPECT_EQ(grid->cgetOrigin().position.x, origin[0]);
  EXPECT_EQ(grid->cgetInfo().width,  width);
  EXPECT_EQ(grid->cgetInfo().height, height);
  EXPECT_EQ(grid->cgetInfo().resolution, resolution);

  nh.deleteParam("resolution");
  nh.deleteParam("origin");
  nh.deleteParam("width");
  nh.deleteParam("height");
  nh.deleteParam("line_width");
  nh.deleteParam("line0");
}

TEST_F(OccupancyGridTest, MakeOccupancyGridNoLines)
{
  double resolution = 1;
  double width      = 50;
  double height     = 50;
  double line_width = 1;
  std::vector<double> origin;
  origin.push_back(1.0);
  origin.push_back(1.0);
  origin.push_back(0.0);

  nh.setParam("resolution", resolution);
  nh.setParam("origin", origin);
  nh.setParam("width", width);
  nh.setParam("height", height);
  nh.setParam("line_width", line_width);

  std::shared_ptr<OccupancyGrid> grid = OccupancyGrid::makeOccupancyGrid(this->nh);

  EXPECT_EQ(grid->cgetOrigin().position.x, origin[0]);
  EXPECT_EQ(grid->cgetInfo().width,  width);
  EXPECT_EQ(grid->cgetInfo().height, height);
  EXPECT_EQ(grid->cgetInfo().resolution, resolution);

  for (auto itr = grid->cgetOccupancyGrid().datastart; itr != grid->cgetOccupancyGrid().dataend; itr++)
  {
    EXPECT_FALSE(*itr);
  }

  nh.deleteParam("resolution");
  nh.deleteParam("origin");
  nh.deleteParam("width");
  nh.deleteParam("height");
  nh.deleteParam("line_width");
}

TEST_F(OccupancyGridTest, MakeOccupancyGridNoTopics)
{
  double resolution = 1;
  double width      = 50;
  double height     = 50;
  double line_width = 1;
  std::vector<double> origin, line;
  origin.push_back(1.0);
  origin.push_back(1.0);
  origin.push_back(0.0);
  line.push_back(10.0);
  line.push_back(0.0);
  line.push_back(20.0);
  line.push_back(0.0);

  nh.setParam("resolution", resolution);
  nh.setParam("origin", origin);
  nh.setParam("width", width);
  nh.setParam("height", height);
  nh.setParam("line_width", line_width);
  nh.setParam("line0", line);

  std::shared_ptr<OccupancyGrid> grid = OccupancyGrid::makeOccupancyGrid(this->nh);

  std::array<double,2> point;
  point[0] = 1.0;
  point[1] = 21.0;

  EXPECT_TRUE(grid->isOccupied(point));
  EXPECT_EQ(grid->cgetOrigin().position.x, origin[0]);
  EXPECT_EQ(grid->cgetInfo().width,  width);
  EXPECT_EQ(grid->cgetInfo().height, height);
  EXPECT_EQ(grid->cgetInfo().resolution, resolution);

  nh.deleteParam("resolution");
  nh.deleteParam("origin");
  nh.deleteParam("width");
  nh.deleteParam("height");
  nh.deleteParam("line_width");
  nh.deleteParam("line0");
}

TEST_F(OccupancyGridTest, MakeOccupancyGridOnlyRequired)
{
  double resolution = 1;
  double width      = 50;
  double height     = 50;
  double line_width = 1;
  std::vector<double> origin;
  origin.push_back(1.0);
  origin.push_back(1.0);
  origin.push_back(0.0);

  nh.setParam("resolution", resolution);
  nh.setParam("origin", origin);
  nh.setParam("width", width);
  nh.setParam("height", height);
  nh.setParam("line_width", line_width);

  std::shared_ptr<OccupancyGrid> grid = OccupancyGrid::makeOccupancyGrid(this->nh);

  EXPECT_EQ(grid->cgetOrigin().position.x, origin[0]);
  EXPECT_EQ(grid->cgetInfo().width,  width);
  EXPECT_EQ(grid->cgetInfo().height, height);
  EXPECT_EQ(grid->cgetInfo().resolution, resolution);

  for (auto itr = grid->cgetOccupancyGrid().datastart; itr != grid->cgetOccupancyGrid().dataend; itr++)
  {
    EXPECT_FALSE(*itr);
  }

  nh.deleteParam("resolution");
  nh.deleteParam("origin");
  nh.deleteParam("width");
  nh.deleteParam("height");
  nh.deleteParam("line_width");
}
