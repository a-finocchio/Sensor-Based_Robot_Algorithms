// Copyright (C) 2008 Wim Meeussen <meeussen at willowgarage com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#include "measurement_pdf.hpp"
#include "ekf_models.hpp"
#include <bfl/wrappers/rng/rng.h> // Wrapper around several rng libraries
#include <tf/tf.h>

namespace BFL
{

  using namespace MatrixWrapper;
  
  MeasurementPDF::MeasurementPDF( const Gaussian& additiveNoise )
    : AnalyticConditionalGaussianAdditiveNoise( additiveNoise ){}
  
  MeasurementPDF::~MeasurementPDF(){}

  ColumnVector MeasurementPDF::ExpectedValueGet() const{

    ColumnVector state = ConditionalArgumentGet(0);

    // copy to the state
    State state_in;
    for( size_t i=1; i<=6; i++ )
      { state_in.x[i-1] = state(i); }

    // Call the state prediction
    sensor_msgs::NavSatFix nsf = meas_evaluate_gps( state_in );
    sensor_msgs::RPY rpy = meas_evaluate_imu( state_in );
    
    // copy to the state
    ColumnVector z(6);
    z(1) = nsf.latitude;
    z(2) = nsf.longitude;
    z(3) = nsf.altitude;

    //tf::Quaternion q;
    //tf::quaternionMsgToTF( imu.orientation, q );
    //tf::Matrix3x3 R( q );
    
    //double r, p, y;
    //R.getRPY( r, p, y );
    
    z(4) = rpy.roll;
    z(5) = rpy.pitch;
    z(6) = rpy.yaw;

    return z + AdditiveNoiseMuGet();

  }
  
  Matrix MeasurementPDF::dfGet(unsigned int i) const{

    Matrix df( 6, 6 );

    // initialize df matrix
    for( int r=1; r<=6; r++){
      for( int c=1; c<=6; c++){
        if( r == c ) { df(r,c) = 1; }
        else         { df(r,c) = 0; }
      }
    }

    if( i==0 ){

      ColumnVector state = ConditionalArgumentGet(0);

      // copy to the state
      State s;
      for( size_t i=1; i<=6; i++ ) { s.x[i-1] = state(i); }

      double Hgps[3][3], Himu[3][3];
      meas_evaluate_Hgps( Hgps, s );
      meas_evaluate_Himu( Himu, s );

      for( int r=1; r<=3; r++){
	for( int c=1; c<=3; c++){
	  df(r,c) = Hgps[r-1][c-1];
	}
      }

      for( int r=4; r<=6; r++){
	for( int c=4; c<=6; c++){
	  df(r,c) = Himu[r-4][c-4];
	}
      }

    }

    return df;

  }

}//namespace BFL

