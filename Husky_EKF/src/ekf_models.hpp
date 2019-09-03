
#ifndef __EKF_MODELS_HPP__
#define __EKF_MODELS_HPP__

#include <iostream>
#include <iomanip>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

namespace sensor_msgs{
  struct RPY{
    double roll;
    double pitch;
    double yaw;
  };
}

struct State{

  enum { POS_X,      // X position
	 POS_Y,      // Y position
	 POS_Z,      // Z position
	 ROT_R,      // rotation about X
	 ROT_P,      // rotation about Y
	 ROT_Y };    // rotation about Z

  double x[6];      // the state vector

  friend std::ostream& operator << ( std::ostream& os, const State& s){

    os << std::setw(15) << "POS_X: " << s.x[POS_X]
       << std::setw(15) << "POS_Y: " << s.x[POS_Y]
       << std::setw(15) << "POS_Z: " << s.x[POS_Z] << std::endl
       << std::setw(15) << "ROT_R: " << s.x[ROT_R] 
       << std::setw(15) << "ROT_P: " << s.x[ROT_P]
       << std::setw(15) << "ROT_Y: " << s.x[ROT_Y] << std::endl;

    return os;

  }

};

/**
   TODO
   Fill in the value of the process covariance matrix. The rows/columns of C are
   in the following order [ POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ]. The 
   performance of your EKF will vary based on the values you provide here.
   \param[out] C Covariance matrix of the process.
*/
void process_covariance( double C[6][6] );

/**
   TODO
   Fill in the value of the measurement covariance matrix. The rows/columns of C
   are in the following order [POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ]. The 
   performance of your EKF will vary based on the values you provide here.
   \param[out] C Covariance matrix of the process.
*/
void measurement_covariance( double C[6][6] );

/**
   TODO
   Evaluate the system function.
   This function returns the prediction of the next state based on the 
   current state estimate and the commmand input (linear/angular velocities).
   \param state_in The current state estimate
   \param delta_v     The input linear position increment
   \param delta_w     The input angular position increment
*/
State sys_evaluate_g( const State& state_in, double delta_v, double delta_w );

/** 
   TODO
   Evaluate the system Jacobian.
   This function evaluates the Jacobian of the system functions g (see 
   sys_evaluate_g). The entry G[i][j] represents ( d g_i / d s_j )
   \param[out] G      The 6x6 Jacobian of the function g
   \param state       The state of the robot
   \param delta_v     The input linear position increment
   \param delta_w     The input angular position increment
*/
void sys_evaluate_G( double G[6][6], const State& state, double delta_v, double delta_w );

/**
   TODO
   Evaluate the GPS observation function.
   This function returns the expected satellite fix given the state of the robot
   \param state_in The state estimate
   \return         A satellite navigation fix (only the latitute, longitude
                   and altitude members are used)
*/
sensor_msgs::NavSatFix meas_evaluate_gps( const State& state );

/**
   Evaluate the IMU observation function.
   This function computes the expected imu orientation given the state of the 
   robot.
   \param state_in The current state estimate
   \return         A inertial navigation unit measurement (only the orientation
                   member is used).
*/
sensor_msgs::RPY meas_evaluate_imu( const State& state );

/** 
    Observation Jacobian of the GPS
    This function returns the 3x3 observation Jacobian of the GPS. Essentially,
    this is the Jacobian of your meas_evaluate_gps function.
    \param[out] Hgps The 3x3 GPS Jacobian.
    \param[in]  state The state of the robot
*/
void meas_evaluate_Hgps( double Hgps[3][3], const State& state );

/** 
    Observation Jacobian of the IMU
    This function returns the 3x3 observation Jacobian of the IMU. Essentially,
    this is the Jacobian of your meas_evaluate_imu function.
    \param[out] Himu The 3x3 IMU Jacobian.
    \param[in]  state The state of the robot
*/
void meas_evaluate_Himu( double Himu[3][3], const State& state );

#endif
