
#ifndef __EKF_NODE__
#define __EKF_NODE__
  
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "ekf.hpp"

// messages
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace message_filters;

class EKFNode{

private:

  // shorthand typedefs
  typedef Subscriber<NavSatFix>    SubscribeGPS;
  typedef Subscriber<Imu>          SubscribeIMU;
  typedef sync_policies::ApproximateTime<NavSatFix,Imu> Policy;
  typedef message_filters::Synchronizer< EKFNode::Policy > Sync;
  
  SubscribeGPS    sub_gps;
  SubscribeIMU    sub_imu;
  Sync            sync;

  BFL::ColumnVector z;

  ros::NodeHandle nh, nh_private;

  ros::Subscriber sub_twist;
  ros::Publisher  pub_pose;

  EKF ekf;
  
public:

  EKFNode( ros::NodeHandle& nh, ros::NodeHandle& nhp );
  
  void callback_sensors( const NavSatFix& nsf, const Imu& imu );
  void callback_command( const Twist& vw );


};

#endif
