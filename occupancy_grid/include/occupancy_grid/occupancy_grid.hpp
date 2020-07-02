/**
 * @File: occupancy_grid.hpp
 * @Date: March 2020
 * @Author: James Swedeen
 *
 * @brief
 * Used to generate, manipulate, and use a occupancy grid.
 **/

#ifndef OCCUPANCY_GRID_OCCUPANCY_GRID_HPP
#define OCCUPANCY_GRID_OCCUPANCY_GRID_HPP

/* Local Headers */
#include"occupancy_grid/GetOccupancyGrid.h"

/* C++ Headers */
#include<cstdint>
#include<vector>
#include<array>
#include<memory>

/* Open CV Headers */
#include<opencv2/opencv.hpp>

/* ROS Headers */
#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/MapMetaData.h>

class OccupancyGrid
{
public:
  /**
   * @Default Constructor
   **/
  OccupancyGrid() = delete;
  /**
   * @Copy Constructor
   **/
  OccupancyGrid(const OccupancyGrid&) = default;
  /**
   * @Move Constructor
   **/
  OccupancyGrid(OccupancyGrid&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the occupancy grid for use.
   * If the lines parameters are provided the constructor will add the specified lines
   * to the grid as occupied pixels.
   * If the topic parameters are provided a ROS service and ROS publisher will be
   * initialized and ready for use.
   *
   * @parameters
   * lines: A list of lines to be set to occupied in the grid
   * line_width: The number of pixels the lines will be thick
   * resolution: The distance each pixel will be from each other in real distance
   * origin: The pose of the lower left hand corner of the grid
   * width: The width of the gird in normal measurements
   * height: The height of the gird in normal measurements
   **/
  OccupancyGrid(const std::vector<std::array<std::array<double,2>,2>>& lines,
                const double                                           line_width,
                const float                                            resolution,
                const geometry_msgs::Pose&                             origin,
                const double                                           width,
                const double                                           height);

  OccupancyGrid(const double               line_width,
                const float                resolution,
                const geometry_msgs::Pose& origin,
                const double               width,
                const double               height);
  /**
   * @Clone Constructor
   *
   * @brief
   * Uses the information in the parameter to initialize a fully operational occupancy
   * grid other then the ROS connections.
   *
   * @parameters
   * info: Information about the occupancy gird this object will become
   **/
  OccupancyGrid(const occupancy_grid::GetOccupancyGridResponse& info);
  /**
   * @Deconstructor
   **/
  ~OccupancyGrid() noexcept = default;
  /**
   * @Assignment Operators
   **/
  OccupancyGrid& operator=(const OccupancyGrid&)  = default;
  OccupancyGrid& operator=(      OccupancyGrid&&) = default;
  /**
   * @makeOccupancyGrid
   *
   * @brief
   * Uses ROS Parameters found in the following locations to construct a OccupancyGrid.
   * For a list of all ROS Parameters needed look at occupancy_grid.launch.
   *
   * @return
   * A fully constructed OccupancyGrid.
   **/
  static std::shared_ptr<OccupancyGrid> makeOccupancyGrid(const ros::NodeHandle& nh);
  /**
   * @isOccupied
   *
   * @brief
   * Tests whether or not a point on the grid is occupied.
   *
   * @parameters
   * point: The x, y coordinates to be tested in normal units
   *
   * @return
   * True if the point is occupied and false otherwise.
   **/
  bool isOccupied(const std::array<double,2>& point) const;
  /**
   * @setPoint
   *
   * @brief
   * Sets a point in the occupancy grid to the specified value.
   *
   * @parameters
   * point: The x, y coordinates to be tested in normal units
   * val: True if you want to set it to occupied and false if you want to
   *      set it not occupied
   **/
  void setPoint(const std::array<double,2>& point, const bool val);
  /**
   * @setLine
   *
   * @brief
   * Sets the line passed in to to the specified value.
   *
   * @parameters
   * line: The x, y coordinates to be tested in normal units
   * val: True if you want to set it to occupied and false if you want to
   *      set it not occupied
   **/
  void setLine(const std::array<std::array<double,2>,2>& line, const bool val);
  /**
   * @set
   *
   * @brief
   * Used to set this object's internally held parameters.
   *
   * @parameters
   * line_width: The width of all new lines added
   * origin: The origin of the grid
   * load_time: The time that the mat was loaded
   *
   * @return
   * The new value.
   **/
  double               setLinewidth(const double               line_width) noexcept;
  geometry_msgs::Pose& setOrigin(   const geometry_msgs::Pose& origin)     noexcept;
  ros::Time            setLoadTime( const ros::Time            load_time)  noexcept;
  /**
   * @cget
   *
   * @brief
   * Used to modify internally held optimization options.
   *
   * @return
   * A reference to the thing that was asked for.
   **/
  const cv::Mat&               cgetOccupancyGrid() const noexcept;
  const nav_msgs::MapMetaData& cgetInfo()          const noexcept;
  const geometry_msgs::Pose&   cgetOrigin()        const noexcept;
        double                 lineWidth()         const noexcept;
        float                  resolution()        const noexcept;
        uint32_t               width()             const noexcept;
        uint32_t               height()            const noexcept;
        double                 xLowerBound()       const noexcept;
        double                 xUpperBound()       const noexcept;
        double                 yLowerBound()       const noexcept;
        double                 yUpperBound()       const noexcept;
private:
  /* Grid Data */
  nav_msgs::MapMetaData info;
  cv::Mat               grid;
  double                line_width;
  /**
   * @getIndex
   *
   * @brief
   * Finds the pixel row and column that corresponds to the x, y point in
   * normal units.
   *
   * @parameters
   * point: The x, y values in normal units
   * row_index: The row index in the grid
   * col_index: The col index in the grid
   **/
  void findIndex(const std::array<double,2>& point, size_t& row_index, size_t& col_index) const;
};

#endif
/* occupancy_grid.hpp */
