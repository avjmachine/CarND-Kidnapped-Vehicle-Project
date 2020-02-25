/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Number of particles set as 100. All particles initialized to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. Also, random Gaussian noise is added to each particle.
   */

  num_particles = 100;  

  std::default_random_engine gen;

  std::normal_distribution<double> x_nd{x,std[0]};
  std::normal_distribution<double> y_nd{y,std[1]};
  std::normal_distribution<double> theta_nd{theta,std[2]};

  for (int i=0; i<num_particles; i++) {
      Particle p;
      
      p.id = i;
      p.x = x_nd(gen);
      p.y = y_nd(gen);
      p.theta = theta_nd(gen);
      p.weight = 1; 
      
      particles.push_back(p);     
      weights.push_back(1);
  }

  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * Measurements of yaw rate and velocity are used to calculate the 
   * position and orientation of each particle using bicycle model equations.
   * Random Gaussian noise is also introduced in the predictions.
   */

   std::default_random_engine gen;
  
   double v_by_thetadot, delta_yaw;

   if (fabs(yaw_rate) >= 1e-5) {
       v_by_thetadot = velocity / yaw_rate;
   }
   
   delta_yaw = yaw_rate * delta_t;

   for (int i=0; i<num_particles; i++) {
       double x_0 = particles[i].x;
       double y_0 = particles[i].y;
       double theta_0 = particles[i].theta;
      
       double x_t, y_t, theta_t;

       if (fabs(yaw_rate) >= 1e-5) {
           theta_t = theta_0 + delta_yaw;
           x_t = x_0 + (v_by_thetadot * (sin(theta_t) - sin(theta_0)));
           y_t = y_0 + (v_by_thetadot * (cos(theta_0) - cos(theta_t)));
       }
       else {
           theta_t = theta_0;
	   x_t = x_0 + (velocity * delta_t * cos(theta_0));
	   y_t = y_0 + (velocity * delta_t * sin(theta_0));
       }

       std::normal_distribution<double> newx(x_t,std_pos[0]);
       std::normal_distribution<double> newy(y_t,std_pos[1]);
       std::normal_distribution<double> newtheta(theta_t,std_pos[2]);
     
       particles[i].x = newx(gen);
       particles[i].y = newy(gen);
       particles[i].theta = newtheta(gen);
   }	   
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * The landmark on map that is closest to each observed measurement is  
   * searched and its ID is assigned to the corresponding observed measurement.
   * Only landmarks within the maximum measured observation range are passed into this function 
   * to conserve calculation time.
   */
   
    for (int i = 0; i < observations.size(); i++) {
        
	double min_dist = std::numeric_limits<double>::infinity();
        int closest_map_landmark_id = 0;
        
	for (int j = 0; j < predicted.size(); j++) {
	    double dist_knn = sqrt(pow((observations[i].x - predicted[j].x),2) + pow((observations[i].y - predicted[j].y),2));
	    if (dist_knn < min_dist) {
	        closest_map_landmark_id = predicted[j].id;
		min_dist = dist_knn;
	    }
	}
	observations[i].id = closest_map_landmark_id;
    }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * For each particle, the observations are transformed from the vehicle coordinate 
   * system to the map coordinate system (using rotation and translation). The distance of 
   * the farthest observation (times 1.1 as error tolerance factor) is used as a filter to 
   * search only for the actual map landmarks within this distance. To each observation, 
   * an associated map landmark is found and its multivariate gaussian probability is computed
   * keeping the positions of the actual landmark as mean (and also using the given standard deviation
   * values). The weight of each particle is calculated by taking a product of these probabilities
   * for each observation. 
   */

    double sigx = std_landmark[0];
    double sigy = std_landmark[1];

    for(int i=0; i<num_particles; i++)  {
	double x_p = particles[i].x;
	double y_p = particles[i].y;
	double theta_p = particles[i].theta;

        vector<LandmarkObs> obs_in_mapcoords;
	double max_landmark_dist = 0.0; 
        
	for(int j = 0; j < observations.size(); j++) {
            LandmarkObs l;
            l.id = 0;  //this id is later modified to the map landmark id using the dataAssociation() function
	    double x_c = observations[j].x;
	    double y_c = observations[j].y;
            l.x = x_p + (cos(theta_p) * x_c) - (sin(theta_p) * y_c);
            l.y = y_p + (sin(theta_p) * x_c) + (cos(theta_p) * y_c);
            obs_in_mapcoords.push_back(l);
	    double dist_from_particle = dist(x_p, l.x, y_p, l.y);
	    //calculation of the distance of the farthest measurement from the particle
	    if (dist_from_particle > max_landmark_dist) {
	        max_landmark_dist = dist_from_particle;
	    }
        }
       
        vector<LandmarkObs> landmarks_in_range;
    
	for(int k=0; k < map_landmarks.landmark_list.size(); k++){
            LandmarkObs l;
	    l.id = map_landmarks.landmark_list[k].id_i;
	    l.x = map_landmarks.landmark_list[k].x_f;
	    l.y = map_landmarks.landmark_list[k].y_f;
	    //only map landmarks within the farthest measurement from the particle are to be searched
	    //this will save computation time. A scale factor of 1.1 is used for accomodating errors.
	    if (dist(x_p,l.x,y_p,l.y) <= 1.1*max_landmark_dist) {
	        landmarks_in_range.push_back(l);  
	    }
        }

        dataAssociation(landmarks_in_range, obs_in_mapcoords);
	
        vector<int> associations;
	vector<double> sense_x, sense_y;
        double product_mvgauss_prob = 1.0;

	for(int j = 0; j < obs_in_mapcoords.size(); j++) {
	    double obsx = obs_in_mapcoords[j].x;
	    double obsy = obs_in_mapcoords[j].y;
	    int map_landmark_id = obs_in_mapcoords[j].id; 
	    double mux = map_landmarks.landmark_list[map_landmark_id-1].x_f; //subtract 1 since map landmark list index starts with 0
	    double muy = map_landmarks.landmark_list[map_landmark_id-1].y_f; //subtract 1 since map landmark list index starts with 0
	    double gauss_const_term = 1/(2*M_PI*sigx*sigy);
	    double gauss_exp_term = (pow((obsx-mux),2)/(2*sigx*sigx))+(pow((obsy-muy),2)/(2*sigy*sigy));
	    // Multivariate Gaussian Probability of each individual landmark measurement
	    double obs_mvgauss_prob = gauss_const_term * exp(-gauss_exp_term);
	    // Combined Gaussian Probability using a product of each individual probability
	    product_mvgauss_prob *= obs_mvgauss_prob;
	    particles[i].weight = product_mvgauss_prob;

	    associations.push_back(map_landmark_id);
	    sense_x.push_back(obsx);
	    sense_y.push_back(obsy);
	}
	SetAssociations(particles[i], associations, sense_x, sense_y);
	weights[i] = particles[i].weight;
    }
}

void ParticleFilter::resample() {
  /**
   * The particles are now resampled - random selection with replacement 
   * with probability proportional to their weight (in a discrete distribution). 
   */

    std::default_random_engine gen;
    std::discrete_distribution<> weight_index_dd (weights.begin(), weights.end());

    vector<Particle> resampled; 
    
    for(int i=0; i<num_particles; i++) {
	int sample = weight_index_dd(gen);    
        resampled.push_back(particles[sample]);
    }	    

    //Update the original set of particles with the resampled particles
    particles = resampled;

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
