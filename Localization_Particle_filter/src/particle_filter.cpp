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

default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
	if(is_initialized)
		return;

	num_particles = 100;
	is_initialized = true;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for(int i = 0; i<num_particles; i++){
		Particle p;
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1.0;
		particles.push_back(p);
		weights.push_back(1);
	}
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
	double std_x, std_y, std_theta;
	std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = std_pos[2];
	normal_distribution<double> dist_x(0, std_x);
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);

	//Calculation simplified
	double v_ =  velocity/yaw_rate;
	double dt_yr_ = yaw_rate * delta_t;
	double v_dt_ = velocity * delta_t;
	
	for(int i=0; i < num_particles; i++){
		if(fabs(yaw_rate) < 0.001) {
			particles[i].x += v_dt_ * cos(particles[i].theta);
			particles[i].y += v_dt_ * sin(particles[i].theta);
		} else {
			particles[i].x += v_ * (sin(particles[i].theta + dt_yr_) - sin(particles[i].theta));
			particles[i].y += v_ * (cos(particles[i].theta) - cos(particles[i].theta + dt_yr_) );
			particles[i].theta += dt_yr_;
		}
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta +=  dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
	for(int i=0; i < observations.size(); i++) {
		LandmarkObs& observation_Object = observations.at(i);
		double min_dist = 0;
		for(int j=0; j < predicted.size(); j++) {
			LandmarkObs& predicted_Object = predicted.at(j);
			double eu_dist = dist(observation_Object.x, observation_Object.y, predicted_Object.x, predicted_Object.y);
			if (j == 0) {
				min_dist = eu_dist;
			} else {
				if (eu_dist < min_dist) {
					min_dist = eu_dist;
					observation_Object.id = predicted_Object.id-1;
				}
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
   
	for(int i=0; i < num_particles; i++) {
		vector<LandmarkObs> landmarks_MapCord;
		
		//load values from landmark list
		for(int j=0; j < map_landmarks.landmark_list.size(); j++) {
			Map::single_landmark_s& sls = map_landmarks.landmark_list.at(j);
			LandmarkObs landmark_Object;
			landmark_Object.id = sls.id_i;
			landmark_Object.x = sls.x_f;
			landmark_Object.y = sls.y_f;
			landmarks_MapCord.push_back(landmark_Object);
		}
		
		Particle& p = particles.at(i);
		vector<LandmarkObs> observations_MapCord;
		
		//transform values from sensor coordinates to map coordinates
		for(int k=0; k < observations.size(); k++) {
			LandmarkObs& observation_Object = observations.at(k);
			LandmarkObs transformed_landmark_Object;
			transformed_landmark_Object.id = observation_Object.id;
			transformed_landmark_Objectv.x = p.x + observation_Object.x * cos(p.theta) - observation_Object.y * sin(p.theta);
			transformed_landmark_Object.y = p.y + observation_Object.x * sin(p.theta) + observation_Object.y * cos(p.theta);
			observations_MapCord.push_back(transformed_landmark_Object);
		}

		//find associated particle, observation mapping - writes correspondency in observation object ID (see dataAssociation function)
		dataAssociation(landmarks_MapCord, observations_MapCord);
		
		//update weights based on the mult-variate function
		double prob = 1.0;
		for(int k=0; k < observations_MapCord.size(); k++) {
			LandmarkObs& observation_Object = observations_MapCord.at(k);
			LandmarkObs& landmark_Object = landmarks_MapCord.at(observation_Object.id);
			double distance = dist(landmark_Object.x, landmark_Object.y, p.x, p.y);

			if (distance > sensor_range)
				continue;

			double mu_x = landmark_Object.x, mu_y = landmark_Object.y;
			double exponent = pow(observation_Object.x - mu_x, 2)/(2 * pow(sig_x, 2)) + pow(observation_Object.y - mu_y, 2)/(2 * pow(sig_y, 2));

			prob *= gauss_norm * exp(-exponent);

			p.weight = prob;
			weights.at(i) = p.weight;
		}
	}

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

	vector<double> weights;
	double mw = numeric_limits<double>::min();

	for(int i =0; i < num_particles; i++){
		weights.push_back(particles[i].weight);
		if(particles[i].weight > mw)
			mw = particles[i].weight;
	}

	random_device seed;
	mt19937 random_generator(seed());
	// sample particles based on their weight
	discrete_distribution<> sample(weights.begin(), weights.end());
	vector<Particle> new_particles(num_particles);
	for(auto & p : new_particles)
		p = particles[sample(random_generator)];
	
	particles = move(new_particles);
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
