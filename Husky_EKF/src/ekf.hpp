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

#ifndef __EKF__
#define __EKF__

// bayesian filtering
#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include "system_pdf.hpp"
#include "measurement_pdf.hpp"

// TF
#include <tf/tf.h>

// msgs
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// log files
#include <fstream>

class EKF{

public:

  /// constructor
  EKF();

  /// destructor
  ~EKF();

  /** update the extended Kalman filter
   * \param filter_time update the ekf up to this time
   * returns true on successfull update
   */
  bool update( const ros::Time&  filter_time, 
	       const BFL::ColumnVector& z,
	       const BFL::ColumnVector& u );


  /** initialize the extended Kalman filter
   * \param prior the prior robot pose
   * \param time the initial time of the ekf
   */
  
  void initialize();
  bool is_initialized() const { return initialized; }

  geometry_msgs::PoseWithCovarianceStamped get_posterior() const;

private:

  // pdf / model / filter
  BFL::AnalyticSystemModelGaussianUncertainty*            sys_model_;
  BFL::SystemPDF*                                         sys_pdf_;

  BFL::AnalyticMeasurementModelGaussianUncertainty*       meas_model_;
  BFL::MeasurementPDF*                                    meas_pdf_;

  BFL::Gaussian*                                          prior_;
  BFL::ExtendedKalmanFilter*                              filter_;

  ros::Time filter_time_old;
  bool initialized;

}; // class

#endif
