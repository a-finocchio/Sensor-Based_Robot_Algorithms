/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include "ekf.hpp"
#include "ekf_models.hpp"

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf;
using namespace std;

// constructor
EKF::EKF():
  prior_( NULL ),
  filter_( NULL ),
  initialized( false ){

  // covariance matrix
  double C[6][6];

  // create system model
  ColumnVector sysNoise_Mu(6);  sysNoise_Mu = 0;
  SymmetricMatrix sysNoise_Cov(6); sysNoise_Cov = 0;
  
  process_covariance( C );
  for( int r=1; r<=6; r++ ){
    for( int c=1; c<=6; c++ ){
      sysNoise_Cov( r, c ) = C[r-1][c-1];
    }
  }
  
  Gaussian system_Uncertainty( sysNoise_Mu, sysNoise_Cov );
  sys_pdf_  = new SystemPDF( system_Uncertainty );
  sys_model_ = new AnalyticSystemModelGaussianUncertainty( sys_pdf_ );
  
  // create measurement model
  ColumnVector measNoise_Mu(6);  measNoise_Mu = 0;
  SymmetricMatrix measNoise_Cov(6);  measNoise_Cov = 0;
  
  measurement_covariance( C );
  for( int r=1; r<=6; r++ ){
    for( int c=1; c<=6; c++ ){
      measNoise_Cov( r, c ) = C[r-1][c-1];
    }
  }

  Gaussian measurement_Uncertainty( measNoise_Mu, measNoise_Cov );
  meas_pdf_   = new MeasurementPDF( measurement_Uncertainty );
  meas_model_ = new AnalyticMeasurementModelGaussianUncertainty( meas_pdf_ );

}

// destructor
EKF::~EKF(){
  if (filter_)      delete filter_;
  if (prior_)       delete prior_;
  if( sys_pdf_)     delete sys_pdf_;
  if( sys_model_)   delete sys_model_;
  if( meas_pdf_ )   delete meas_pdf_;
  if( meas_model_ ) delete meas_model_;
}

// initialize prior density of filter 
void EKF::initialize(){

  ColumnVector prior_Mu(6);
  prior_Mu = 0.0;
  prior_Mu(3) = 0.23;

  // set a good covariance
  SymmetricMatrix prior_Cov(6);
  for (unsigned int i=1; i<=6; i++){
    for (unsigned int j=1; j<=6; j++){
      if (i==j) { prior_Cov(i,j) = pow( 0.001, 2 ); }
      else      { prior_Cov(i,j) = 0; }
    }
  }

  prior_  = new Gaussian( prior_Mu, prior_Cov );
  filter_ = new ExtendedKalmanFilter( prior_ );
  
  filter_time_old = ros::Time::now();
  initialized = true;

}

// return the posterior estimate
geometry_msgs::PoseWithCovarianceStamped EKF::get_posterior() const {

  ColumnVector mu = filter_->PostGet()->ExpectedValueGet();
  
  geometry_msgs::PoseWithCovarianceStamped Rt;
  Rt.header.stamp = ros::Time::now();
  Rt.header.frame_id = "map";
  Rt.pose.pose.position.x = mu(1);
  Rt.pose.pose.position.y = mu(2);
  Rt.pose.pose.position.z = mu(3);

  tf::Quaternion q;
  q.setRPY( mu(4), mu(5), mu(6) );
  Rt.pose.pose.orientation.x = q.getX();
  Rt.pose.pose.orientation.y = q.getY();
  Rt.pose.pose.orientation.z = q.getZ();
  Rt.pose.pose.orientation.w = q.getW();

  SymmetricMatrix C = filter_->PostGet()->CovarianceGet();
  for(unsigned int i=0; i<6; i++)
    for(unsigned int j=0; j<6; j++)
      Rt.pose.covariance[6*i+j] = C(i+1,j+1);

  return Rt;

}

// update filter
bool EKF::update( const ros::Time& filter_time,
		  const ColumnVector& z,
		  const ColumnVector& u ){

  // only update filter for time later than current filter time
  double dt = (filter_time - filter_time_old).toSec();
  if (dt == 0) return false;
  if (dt <  0){
    ROS_INFO("Will not update robot pose with time %f sec in the past.", dt);
    return false;
  }
  ROS_DEBUG("Update filter at time %f with dt %f", filter_time.toSec(), dt);
  
  filter_->Update( sys_model_, u*dt, meas_model_, z );
  filter_time_old = filter_time;

  return true;
}
