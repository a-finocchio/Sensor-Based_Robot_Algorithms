#include "ekf_node.hpp"

// constructor
EKFNode::EKFNode( ros::NodeHandle& nh, ros::NodeHandle& nhp ) : 
  nh( nh ),
  nh_private( nhp ),
  sub_gps( nh, "fix", 2 ),
  sub_imu( nh, "imu", 2 ),
  sync( Policy(20), sub_gps, sub_imu ),
  z( 6 ){

  sync.registerCallback( &EKFNode::callback_sensors, this );

  // advertise our estimation
  pub_pose=nh_private.advertise<PoseWithCovarianceStamped>("odom_combined",10);
  sub_twist=nh.subscribe( "cmd_vel", 2, &EKFNode::callback_command, this );

}

// Sensor callback
// This only copies the measurement
void EKFNode::callback_sensors( const NavSatFix& nsf, const Imu& imu ){

  // combine the sensors
  z(1) = nsf.latitude;
  z(2) = nsf.longitude;
  z(3) = nsf.altitude;

  tf::Quaternion q;
  tf::quaternionMsgToTF( imu.orientation, q );
  tf::Matrix3x3 R( q );

  double r, p, y;
  R.getRPY( r, p, y );

  z(4) = r;
  z(5) = p;
  z(6) = y;

}

// Velocity command callback
// This calls the EKF with the latest measurements
void EKFNode::callback_command( const Twist& vw ){

  BFL::ColumnVector u(2);
  u(1) = vw.linear.x;
  u(2) = vw.angular.z;

  if( ekf.is_initialized() ){ 
    ekf.update( ros::Time::now(), z, u ); 
    pub_pose.publish(  ekf.get_posterior() );
  }
  else{ ekf.initialize(); }

}


int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "EKF");
  ros::NodeHandle nh, nhp("~");

  // create filter class
  EKFNode ekf_node( nh, nhp );

  ros::spin();
  
  return 0;
}
