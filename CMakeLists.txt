cmake_minimum_required(VERSION 2.8.3)
project(cam_grid)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  camera_info_manager
  image_geometry
)


###########
## Build ##
###########

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/flycapture
	/home/rrc/catkin_ws/src/cam_grid/include
)
## Declare a C++ executable
add_executable(grid_cam src/grid_cam.cpp src/INIReader.cpp)

target_link_libraries(grid_cam ${catkin_LIBRARIES} flycapture /home/rrc/catkin_ws/src/cam_grid/lib/libinih.a)


