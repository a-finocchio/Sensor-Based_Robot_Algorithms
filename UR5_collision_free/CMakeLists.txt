cmake_minimum_required(VERSION 2.8.3)
project(assignment3)

add_definitions(-std=c++11)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package( catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  roscpp
  rosconsole
  pluginlib
  tf
  dynamic_reconfigure
  eigen_conversions
  ur_description)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})
include_directories( ${catkin_INCLUDE_DIRS} )

link_directories(${Boost_LIBRARY_DIRS})

add_library( assignment3_context src/assignment3_context.cpp )
target_link_libraries( assignment3_context ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library( assignment3_plugin src/assignment3_plugin.cpp)
target_link_libraries( assignment3_plugin assignment3_context ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

