
#include "ekf_models.hpp"
#include <tf/tf.h>

/**
   TODO
   Fill in the value of the process covariance matrix. The rows/columns of C are
   in the following order [POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ].
   \param[out] C Covariance matrix of the process.
*/
void process_covariance( double C[6][6] ){

  for( int r=0; r<6; r++ )
    for( int c=0; c<6; c++ )
      C[r][c] = 0.0;
  
  // TODO fill in the matrix C

  C[0][0]=0.001;
  C[1][1]=0.001;
  C[2][2]=0.001;
  C[3][3]=0.001;
  C[4][4]=0.001;
  C[5][5]=0.03;
}

/**
   TODO
   Fill in the value of the measurement covariance matrix. The rows/columns of C
   are in the following order [POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ]
   \param[out] C Covariance matrix of the process.
*/
void measurement_covariance( double C[6][6] ){

  for( int r=0; r<6; r++ )
    for( int c=0; c<6; c++ )
      C[r][c] = 0.0;

  C[0][0]=6.5e-13;
  C[1][1]=7.5e-13;
  C[2][2]=1.00;
  C[3][3]=2.25e-11;
  C[4][4]=2.15e-12;
  C[5][5]=0.0395e-10;
}



/**
   TODO
   Evaluate the system function.
   Compute the process model.
   This function returns the prediction of the next state based on the 
   current state estimate and the commmand input (linear/angular velocities).
   \param state_in    The current state estimate
   \param delta_v     The input linear position increment
   \param delta_w     The input angular position increment
*/
State sys_evaluate_g( const State& state_in, double dv, double dw ){

  State state_out;
  // TODO
  // predict state_out based on state_in, dv, and dw
  double x = state_in.x[0];
  double y = state_in.x[1];
  double z = state_in.x[2];
  double Roll = state_in.x[3];
  double Pitch = state_in.x[4];
  double Yaw = state_in.x[5];

  state_out.x[0] = x+dv*cos(Pitch)*cos(Yaw);
  state_out.x[1] = y+dv*cos(Pitch)*sin(Yaw);
  state_out.x[2] = z-dv*sin(Pitch);
  state_out.x[3] = atan2(sin(Pitch)*sin(dw) + cos(Pitch)*sin(Roll)*cos(dw), cos(Pitch)*cos(Roll));
  state_out.x[4] = atan2(sin(Pitch)*cos(dw) - cos(Pitch)*sin(Roll)*sin(dw), pow((pow((sin(Pitch)*sin(dw) + cos(Pitch)*sin(Roll)*cos(dw)),2.0) + pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)),(1/2)));
  //atan2(sin(Pitch)*cos(dw) - cos(Pitch)*sin(Roll)*sin(dw), pow(((sin(Pitch)*sin(dw) + cos(Pitch)*sin(Roll)*cos(dw)),2.0) + pow(pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)),0.5);
  state_out.x[5] = atan2(sin(dw)*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) + cos(Pitch)*cos(dw)*sin(Yaw), cos(Pitch)*cos(Yaw)*cos(dw) - sin(dw)*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)));

  return state_out;

}


//atan2(sin(Pitch)*cos(dw) - cos(Pitch)*sin(Roll)*sin(dw), ((sin(Pitch)*sin(dw) + cos(Pitch)*sin(Roll)*cos(dw))^2 + cos(Pitch)^2*cos(Roll)^2)^(1/2))
//atan2(sin(Pitch)*cos(dw) - cos(Pitch)*sin(Roll)*sin(dw), pow((pow((sin(Pitch)*sin(dw) + cos(Pitch)*sin(Roll)*cos(dw)),2.0) + pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)),(1/2)))


//x + dv*cos(Pitch)*cos(Yaw)
//y + dv*cos(Pitch)*sin(Yaw)
//z - dv*sin(Pitch)
//atan2(sin(Pitch)*sin(dw) + cos(Pitch)*sin(Roll)*cos(dw), cos(Pitch)*cos(Roll))
//atan2(sin(Pitch)*cos(dw) - cos(Pitch)*sin(Roll)*sin(dw), ((sin(Pitch)*sin(dw) + cos(Pitch)*sin(Roll)*cos(dw))^2 + cos(Pitch)^2*cos(Roll)^2)^(1/2))
//atan2(sin(dw)*(cos(Roll)*cos(Yaw) + sin(Pitch)*sin(Roll)*sin(Yaw)) + cos(Pitch)*cos(dw)*sin(Yaw), cos(Pitch)*cos(Yaw)*cos(dw) - sin(dw)*(cos(Roll)*sin(Yaw) - cos(Yaw)*sin(Pitch)*sin(Roll)))






/**
   TODO
   Evaluate the system Jacobian.
   This function evaluates the Jacobian of the system functions g (see 
   sys_evaluate_g). The entry G[i][j] represents ( d g_i / d s_j )
   \param[out] G      The 6x6 Jacobian of the function g
   \param      state  The state of the robot
   \param      v      The input linear position increment
   \param      w      The input angular position increment
*/
void sys_evaluate_G( double G[6][6], const State& state, double v, double w ){
  
  // TODO
  // Fill in matrix G
  double Roll = state.x[3];
  double Pitch = state.x[3];
  double Yaw = state.x[3];
  double dw = w;
  double dv = v;

  for (int i=0; i<6; i++){
    for(int j=0; j<6; j++){
      G[i][j]=0;
    }
  }

  G[0][0] = 1.0;
  G[0][4] = -dv*cos(Yaw)*sin(Pitch);
  G[0][5] = -dv*cos(Pitch)*sin(Yaw);
  G[1][1] = 1.0;
  G[1][4] = -dv*sin(Pitch)*sin(Yaw);
  G[1][5] = dv*cos(Pitch)*cos(Yaw);
  G[2][2] = 1.0;
  G[2][4] = -dv*cos(Pitch);
  G[3][3] = (cos(Pitch)*(cos(Pitch)*cos(dw)+sin(Pitch)*sin(Roll)*sin(dw)))/(pow(sin(Pitch)*sin(dw)+cos(Pitch)*sin(Roll)*cos(dw),2.0)+pow(cos(Pitch),2.0)*pow(cos(Roll),2.0));
  G[3][4] = (pow(cos(Pitch),2.0)*pow(cos(Roll),2.0)*((cos(Pitch)*sin(dw)-sin(Pitch)*sin(Roll)*cos(dw))/(cos(Pitch)*cos(Roll))+(1.0/pow(cos(Pitch),2.0)*sin(Pitch)*(sin(Pitch)*sin(dw)+cos(Pitch)*sin(Roll)*cos(dw)))/cos(Roll)))/(pow(sin(Pitch)*sin(dw)+cos(Pitch)*sin(Roll)*cos(dw),2.0)+pow(cos(Pitch),2.0)*pow(cos(Roll),2.0));
  G[4][3] = -cos(Pitch)*cos(Roll)*sin(dw)*1.0/sqrt(pow(sin(Pitch)*sin(dw)+cos(Pitch)*sin(Roll)*cos(dw),2.0)+pow(cos(Pitch),2.0)*pow(cos(Roll),2.0));
  G[4][4] = 1.0/sqrt(pow(sin(Pitch)*sin(dw)+cos(Pitch)*sin(Roll)*cos(dw),2.0)+pow(cos(Pitch),2.0)*pow(cos(Roll),2.0))*(cos(Pitch)*cos(dw)+sin(Pitch)*sin(Roll)*sin(dw));
  G[5][3] = -(sin(Pitch)-sin(Pitch)*pow(cos(dw),2.0)+cos(Pitch)*sin(Roll)*cos(dw)*sin(dw))/(pow(cos(Pitch),2.0)*pow(cos(dw),2.0)+pow(cos(Roll),2.0)*pow(sin(dw),2.0)+pow(sin(Pitch),2.0)*pow(sin(Roll),2.0)*pow(sin(dw),2.0)+cos(Pitch)*sin(Pitch)*sin(Roll)*cos(dw)*sin(dw)*2.0);
  G[5][4] = (cos(Roll)*sin(dw)*(sin(Pitch)*cos(dw)-cos(Pitch)*sin(Roll)*sin(dw)))/(pow(cos(Pitch),2.0)*pow(cos(dw),2.0)+pow(cos(Roll),2.0)*pow(sin(dw),2.0)+pow(sin(Pitch),2.0)*pow(sin(Roll),2.0)*pow(sin(dw),2.0)+cos(Pitch)*sin(Pitch)*sin(Roll)*cos(dw)*sin(dw)*2.0);
  G[5][5] = 1.0;
}

/**
   TODO
   Evaluate the GPS observation function.
   This function returns the expected satellite fix given the state of the robot
   \param state The state estimate
   \return      A satellite navigation fix (only the latitute, longitude
                and altitude members are used)
*/
sensor_msgs::NavSatFix meas_evaluate_gps( const State& state ){

  sensor_msgs::NavSatFix nsf;

  // TODO
  // Convert state to a NavSatFix
  double x=state.x[0];
  double y=state.x[1];
  double z=state.x[2];

  nsf.latitude=x*0.898749804*1e-5;//要改scalar
  nsf.longitude=y*(-0.894774443)*1e-5;//要改scalar
  nsf.altitude=z;
  return nsf;
}

/**
   TODO
   Evaluate the IMU observation function.
   This function computes the expected imu orientation given the state of the 
   robot.
   \param state_in The current state estimate
   \return         A inertial navigation unit measurement (only the orientation
                   member is used).
*/
sensor_msgs::RPY meas_evaluate_imu( const State& state ){
  sensor_msgs::RPY rpy;

  rpy.roll=state.x[3];
  rpy.pitch=state.x[4];
  rpy.yaw=state.x[5];
  
  return rpy;
}

/** 
    TODO
    Observation Jacobian of the GPS
    This function returns the 3x3 observation Jacobian of the GPS. Essentially,
    this is the Jacobian of your meas_evaluate_gps function.
    \param[out] Hgps The 3x3 GPS Jacobian.
    \param[in]  state The state of the robot
*/
void meas_evaluate_Hgps( double Hgps[3][3], const State& state ){

  // TODO
  // Fill the matrix Hgps
  for (int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      Hgps[i][j]=0;
    }
  }

  Hgps[0][0]=0.898749804*1e-5;//scalar
  Hgps[1][1]=(-0.894774443)*1e-5;//scalar
  Hgps[2][2]=1;//scalar
}

/** 
    Observation Jacobian of the IMU
    This function returns the 3x3 observation Jacobian of the IMU. Essentially,
    this is the Jacobian of your meas_evaluate_imu function.
    \param[out] Himu The 3x3 IMU Jacobian.
    \param[in]  state The state of the robot
*/
void meas_evaluate_Himu( double Himu[3][3], const State& state ){

  // TODO
  // Fill the matrix Himu

  for (int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      Himu[i][j]=0;
    }
  }

  Himu[0][0]=1;
  Himu[1][1]=1;
  Himu[2][2]=1;
}

