/**
   likelihood_field
   Implement the likelihood field model for a laser range finder. The function
   returns the probability P( z | s, m ), that is the probability that a 
   measurement z was caused by a robot in a state s and in a map m.
   \input Scan z: A vector of laser beams. Each beam has two members: range and 
                  bearing. You can access the range if the ith beam with
		  z[i].range and the bearing of the ith beam with z[i].bearing.
   \input State s: The state of the robot with respect to the world frame.
                   The x coordinate is given by s[0], the y coordinate by s[1]
		   and the angle theta by s[2].
   \input Map* m:  The map of the environment the robot.
   \input Pose laser_pose: The position and orientation of the laser with 
                           respect to the robot's coordinate frame. The x, y
			   and angle are given by coordinate is laser_pose[0]
			   laser_pose[1] and laser_pose[2] respectively.
   \input sigma_hit: The standard variation of errors of a beam
   \input z_max:  The laser maximum range (in meters)
   \input w_hit:  The coefficient of measurements
   \input w_rand: The coefficient of random errors
   \return        The probability p( z | s, m )
*/

#include <math.h>

double likelihood_field( Scan& z,
			 State s, 
			 Map* m,
			 Pose laser_pose, 
			 double sigma_hit, 
			 double z_max, 
			 double w_hit, 
			 double w_rand,
			 double w_max ) {

    double q = 1.0;
    double k_max = z.size();
    for ( int i = 0; i < k_max; i++) {
        if (z[i].range < z_max) {

            double x_zkt = s[0] + laser_pose[0] * cos(s[2]) - laser_pose[1] * sin(s[2]) + z[i].range * cos(s[2] + laser_pose[2] + z[i].bearing);
            double y_zkt = s[1] + laser_pose[1] * cos(s[2]) + laser_pose[0] * sin(s[2]) + z[i].range * sin(s[2] + laser_pose[2] + z[i].bearing);
            double d2 = pow(GetDistanceFromMap(m, x_zkt, y_zkt), 2);
            q = q * (w_hit * (1/sqrt(2*M_PI*sigma_hit*sigma_hit)*exp(-pow(d2,2.0)/(2*pow(sigma_hit,2.0)))) + w_rand * (1 / z_max));

        }
    }
    return q;
}
















