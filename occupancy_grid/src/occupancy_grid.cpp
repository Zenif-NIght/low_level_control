/**
 * @File: occupancy_grid.cpp
 * @Date: March 2020
 * @Author: James Swedeen
 **/

/* Local Headers */
#include"occupancy_grid/occupancy_grid.hpp"
#include"occupancy_grid/GetOccupancyGrid.h"

/* C++ Headers */
#include<cstdint>
#include<vector>
#include<array>
#include<memory>
#include<cmath>
#include<stdexcept>
#include<algorithm>

/* Open CV Headers */
#include<opencv2/opencv.hpp>

/* ROS Headers */
#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/MapMetaData.h>

/* TF Headers */
#include<tf/tf.h>

OccupancyGrid::OccupancyGrid(const std::vector<std::array<std::array<double,2>,2>>& lines,
                             const double                                           line_width,
                             const float                                            resolution,
                             const geometry_msgs::Pose&                             origin,
                             const double                                           width,
                             const double                                           height)
 : OccupancyGrid(line_width, resolution, origin, width, height)
{
  // Add the lines
  for(auto line_it = lines.cbegin(); line_it != lines.cend(); line_it++)
  {
    this->setLine(*line_it, true);
  }
}

OccupancyGrid::OccupancyGrid(const double               line_width,
                             const float                resolution,
                             const geometry_msgs::Pose& origin,
                             const double               width,
                             const double               height)
 : line_width(line_width)
{
  this->info.map_load_time = ros::Time::now();
  this->info.resolution    = resolution;
  this->info.width         = std::round(width / resolution);
  this->info.height        = std::round(height / resolution);
  this->info.origin        = origin;

  /* Make the grid */
  this->grid = cv::Mat::zeros(this->width(), this->height(), CV_8UC1);
}

OccupancyGrid::OccupancyGrid(const occupancy_grid::GetOccupancyGridResponse& info)
 : info( info.grid.info),
   grid(this->width(), this->height(), CV_8UC1)
{
  std::memcpy(this->grid.data, info.grid.data.data(), info.grid.data.size());
  this->grid = this->grid.t();
}

std::shared_ptr<OccupancyGrid> OccupancyGrid::makeOccupancyGrid(const ros::NodeHandle& nh)
{
  double                      line_width = 0;
  float                       resolution = 0;
  geometry_msgs::Pose         origin;
  double                      width = 0;
  double                      height = 0;

  std::vector<std::array<std::array<double,2>,2>> lines;

  std::vector<double> temp_line;
  bool                has_lines  = false;

  // Get required parameters
  if(!nh.getParam("line_width", line_width))
  {
    throw std::runtime_error("Can't find the line_width parameter");
  }
  if(!nh.getParam("resolution", resolution))
  {
    throw std::runtime_error("Can't find resolution parameter");
  }
  if(!nh.getParam("origin", temp_line))
  {
    throw std::runtime_error("Can't find origin parameter");
  }
  if(!nh.getParam("width", width))
  {
    throw std::runtime_error("Can't find width parameter");
  }
  if(!nh.getParam("height", height))
  {
    throw std::runtime_error("Can't find height parameter");
  }

  ROS_ASSERT(temp_line.size() == 3);
  origin.position.x  = temp_line[0];
  origin.position.y  = temp_line[1];
  origin.position.z  = 0;
  origin.orientation = tf::createQuaternionMsgFromYaw(temp_line[2]);
  temp_line.clear();

  // Get line parameters
  for(uint64_t line_it = 0; nh.getParam("line" + std::to_string(line_it), temp_line); line_it++)
  {
    ROS_ASSERT(temp_line.size() == 4);
    has_lines = true;

    lines.emplace_back();
    lines.back()[0][0] = temp_line[0];
    lines.back()[0][1] = temp_line[1];
    lines.back()[1][0] = temp_line[2];
    lines.back()[1][1] = temp_line[3];
    temp_line.clear();
  }

  // Make the OccupancyGrid
  if(has_lines)
  {
    return std::make_shared<OccupancyGrid>(lines,
                                           line_width,
                                           resolution,
                                           origin,
                                           width,
                                           height);
  }
  else
  {
    return std::make_shared<OccupancyGrid>(line_width,
                                           resolution,
                                           origin,
                                           width,
                                           height);
  }
}

bool OccupancyGrid::isOccupied(const std::array<double,2>& point) const
{
  size_t row_index = -1;
  size_t col_index = -1;

  this->findIndex(point, row_index, col_index);

  return (0 != this->cgetOccupancyGrid().at<uint8_t>(row_index, col_index));
}

void OccupancyGrid::setPoint(const std::array<double,2>& point, const bool val)
{
  size_t row_index = -1;
  size_t col_index = -1;

  this->findIndex(point, row_index, col_index);

  this->grid.at<uint8_t>(row_index, col_index) = val * 100;
}

void OccupancyGrid::setLine(const std::array<std::array<double,2>,2>& line, const bool val)
{
  cv::line(this->grid,
           cv::Point2d(line[0][0] / this->resolution(), line[0][1] / this->resolution()),
           cv::Point2d(line[1][0] / this->resolution(), line[1][1] / this->resolution()),
           100 * val,
           this->lineWidth()/this->resolution(),
           cv::LINE_8);
}

double OccupancyGrid::setLinewidth(const double line_width) noexcept
{
  return (this->line_width = line_width);
}

geometry_msgs::Pose& OccupancyGrid::setOrigin(const geometry_msgs::Pose& origin) noexcept
{
  return (this->info.origin = origin);
}

ros::Time OccupancyGrid::setLoadTime(const ros::Time load_time) noexcept
{
  return (this->info.map_load_time = load_time);
}

const cv::Mat& OccupancyGrid::cgetOccupancyGrid() const noexcept
{
  return this->grid;
}

const nav_msgs::MapMetaData& OccupancyGrid::cgetInfo() const noexcept
{
  return this->info;
}

const geometry_msgs::Pose& OccupancyGrid::cgetOrigin() const noexcept
{
  return this->cgetInfo().origin;
}

double OccupancyGrid::lineWidth() const noexcept
{
  return this->line_width;
}

float OccupancyGrid::resolution() const noexcept
{
  return this->cgetInfo().resolution;
}

uint32_t OccupancyGrid::width() const noexcept
{
  return this->cgetInfo().width;
}

uint32_t OccupancyGrid::height() const noexcept
{
  return this->cgetInfo().height;
}

double OccupancyGrid::xLowerBound() const noexcept
{
  return this->cgetOrigin().position.x;
}

double OccupancyGrid::xUpperBound() const noexcept
{
  return this->cgetOrigin().position.x + (this->width() * this->resolution());
}

double OccupancyGrid::yLowerBound() const noexcept
{
  return this->cgetOrigin().position.y;
}

double OccupancyGrid::yUpperBound() const noexcept
{
  return this->cgetOrigin().position.y + (this->height() * this->resolution());
}

void OccupancyGrid::findIndex(const std::array<double,2>& point, size_t& row_index, size_t& col_index) const
{
  std::array<double,2> shifted_point;

  // Translate to where it is on the grid
  shifted_point[0] = point[0] - this->cgetOrigin().position.x;
  shifted_point[1] = point[1] - this->cgetOrigin().position.y;

  // Get it into the same units as the grid
  shifted_point[0] /= double(this->resolution());
  shifted_point[1] /= double(this->resolution());

  // Round
  row_index = std::floor(shifted_point[0]);
  col_index = std::floor(shifted_point[1]);

  // Make sure it falls on the grid
  row_index = std::max<size_t>(row_index, 0);
  col_index = std::max<size_t>(col_index, 0);

  row_index = std::min<size_t>(row_index, this->cgetOccupancyGrid().rows-1);
  col_index = std::min<size_t>(col_index, this->cgetOccupancyGrid().cols-1);
}

/* occupancy_grid.cpp */
