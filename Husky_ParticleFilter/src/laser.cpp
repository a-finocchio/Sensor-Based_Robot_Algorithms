/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL laser routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_laser.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <iostream>
#include <vector>

#include "laser.h"

////////////////////////////////////////////////////////////////////////////////
// Default constructor
LikelihoodFieldLaser::LikelihoodFieldLaser(size_t max_beams, map_t* map) : 
  amcl::AMCLSensor(), 
  max_samples(0), max_obs(0), 
  temp_obs(NULL)
{
  this->time = 0.0;

  this->max_beams = max_beams;
  this->map = map;

  return;
}

LikelihoodFieldLaser::~LikelihoodFieldLaser()
{
  if(temp_obs){
	for(int k=0; k < max_samples; k++){
	  delete [] temp_obs[k];
	}
	delete []temp_obs; 
  }
}

void 
LikelihoodFieldLaser::SetModelLikelihoodField(double z_hit,
					      double z_rand,
					      double sigma_hit,
					      double max_occ_dist)
{
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;

  map_update_cspace(this->map, max_occ_dist);
}

////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool LikelihoodFieldLaser::UpdateSensor(pf_t *pf, amcl::AMCLSensorData *data)
{
  if (this->max_beams < 2)
    return false;

  pf_update_sensor(pf, (pf_sensor_model_fn_t)LikelihoodFieldModel, data);  

  return true;
}

typedef double State[3];
typedef double Pose[3];

struct Beam{
  double range;
  double bearing;
};

typedef std::vector<Beam> Scan;
typedef map_t Map;


double GetDistanceFromMap( Map* map, double x, double y ){

  // Convert to map grid coords.
  int mi, mj;
  mi = MAP_GXWX(map, x);
  mj = MAP_GYWY(map, y);

  // Part 1: Get distance from the hit to closest obstacle.
  // Off-map penalized as max distance
  double z;
  if(!MAP_VALID(map, mi, mj))
    z = map->max_occ_dist;
  else
    z = map->cells[MAP_INDEX(map,mi,mj)].occ_dist;
  return z;
}

#include "likelihoodfield.cpp"

double LikelihoodFieldLaser::LikelihoodFieldModel( LikelihoodFieldData *data, 
						   pf_sample_set_t* set ){

  LikelihoodFieldLaser* self = (LikelihoodFieldLaser*) data->sensor;
  double total_weight=0.0;

  Scan scan(data->range_count);
  for( size_t i=0; i<scan.size(); i++ ){
    scan[i].range = data->ranges[i][0];
    scan[i].bearing = data->ranges[i][1];
  }

  for( size_t i=0; i<set->sample_count; i++ ){
    double sigma_hit = self->sigma_hit;
    double z_hit = self->z_hit;
    double z_rand = self->z_rand;
    double z_max = self->z_max;
    double range_max = data->range_max;

    pf_sample_t* sample = set->samples + i;
    double w = likelihood_field( scan,
				 sample->pose.v,
				 self->map,
				 self->laser_pose.v,
				 sigma_hit,
				 range_max,
				 z_hit,
				 z_rand,
				 z_max );
    sample->weight *= w;
    total_weight += sample->weight;
  }

  return(total_weight);
}


void LikelihoodFieldLaser::reallocTempData(int new_max_samples, int new_max_obs){
  if(temp_obs){
    for(int k=0; k < max_samples; k++){
      delete [] temp_obs[k];
    }
    delete []temp_obs; 
  }
  max_obs = new_max_obs; 
  max_samples = fmax(max_samples, new_max_samples); 

  temp_obs = new double*[max_samples]();
  for(int k=0; k < max_samples; k++){
    temp_obs[k] = new double[max_obs]();
  }
}
