cmake_minimum_required(VERSION 3.0.2)
project(occupancy_grid)

add_compile_options(-std=c++17)
add_compile_options(-g -Og)
#add_compile_options(-O3)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    tf
    message_generation
    sensor_msgs
    nav_msgs
    geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(GTest  REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
    GetOccupancyGrid.srv
)

generate_messages(
  DEPENDENCIES
    nav_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    occupancy_grid
  CATKIN_DEPENDS
    roscpp
    tf
    message_runtime
    sensor_msgs
    nav_msgs
  DEPENDS
    OpenCV
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

# OccupancyGrid Class
add_library(${PROJECT_NAME}
  src/occupancy_grid.cpp
  src/laser_scan_processor.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Occupancy Grid Node
add_executable(${PROJECT_NAME}_node
  src/occupancy_grid_node.cpp
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

catkin_add_gtest(${PROJECT_NAME}_test test/${PROJECT_NAME}_test.cpp)
if(TARGET ${PROJECT_NAME}_test)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pthread)
endif()

