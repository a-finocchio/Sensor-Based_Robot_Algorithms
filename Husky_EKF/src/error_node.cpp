#include <ros/ros.h>

#include <tf/tf.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <iostream>

geometry_msgs::Point gaz_xyz;
geometry_msgs::Point ekf_xyz;


void gaz_callback( const gazebo_msgs::LinkStates& sl ){
  gaz_xyz = sl.pose[1].position;
}

void ekf_callback( const geometry_msgs::PoseWithCovarianceStamped& p ){
  ekf_xyz = p.pose.pose.position;
}

int main( int argc, char** argv ){

  ros::init( argc, argv, "error_node" );

  ros::NodeHandle nh;

  ros::Subscriber sub_gaz, sub_ekf;
  sub_gaz = nh.subscribe( "/gazebo/link_states", 1, &gaz_callback );
  sub_ekf = nh.subscribe( "/ekf/odom_combined", 1, &ekf_callback );

  ros::Rate rate( 10 );
  while( nh.ok() ){

    std::cout << std::setw(15) << gaz_xyz.x - ekf_xyz.x 
	      << std::setw(15) << gaz_xyz.y - ekf_xyz.y 
	      << std::setw(15) << gaz_xyz.z - ekf_xyz.z << std::endl;
    
    ros::spinOnce();
    rate.sleep();

  }


  return 0;

}
