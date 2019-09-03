cmake_minimum_required(VERSION 2.8.3)
project(assignment5)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  husky_navigation
  roscpp
  amcl
)

find_package(Boost REQUIRED)

catkin_package()

include_directories( ${catkin_INCLUDE_DIRS} )

add_executable( amcl src/amcl_node.cpp src/laser.cpp )
target_link_libraries( amcl ${Boost_LIBRARIES} ${catkin_LIBRARIES} )


