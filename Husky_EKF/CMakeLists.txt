cmake_minimum_required(VERSION 2.8.3)
project(assignment4)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  husky_gazebo
  roscpp
  tf
  nav_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  message_filters
  )

find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)


## System dependencies are found with CMake's conventions
#find_package( Boost REQUIRED COMPONENTS graph thread signals system )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
#add_service_files( FILES PathPlanner.srv )

## Generate added messages and services with any dependencies listed here
#generate_messages( DEPENDENCIES geometry_msgs nav_msgs )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES assignment4
#  CATKIN_DEPENDS husky_gazebo rviz
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
#  ${Boost_INCLUDE_DIRS}
  {BFL_INCLUDE_DIRS}
)

link_directories(${BFL_LIBRARY_DIRS})


## Declare a cpp library
add_library( assignment4
  src/system_pdf.cpp
  src/measurement_pdf.cpp
  src/ekf_models.cpp
  src/ekf.cpp )

## Declare a cpp executable
#add_executable( planner_node src/planner_node.cpp )
#add_executable( planner_client src/planner_client.cpp )
#add_executable( wgs84_node src/wgs84_node.cpp )
add_executable( ekf_node src/ekf_node.cpp )
add_executable( error_node src/error_node.cpp )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies( planner_node assignment4_generate_messages_cpp)
#add_dependencies( planner_client assignment4_generate_messages_cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries( planner_node ${catkin_LIBRARIES} )
#target_link_libraries( planner_client ${catkin_LIBRARIES} )
#target_link_libraries( wgs84_node ${catkin_LIBRARIES} )
target_link_libraries( ekf_node assignment4 ${catkin_LIBRARIES} ${BFL_LIBRARIES} )
target_link_libraries( error_node assignment4 ${catkin_LIBRARIES} ${BFL_LIBRARIES} )
