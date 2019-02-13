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
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
if(is_initialized)
		return;

    num_particles = 200;
	default_random_engine gen;
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
    int nearest_obs_id;
    double max_dist;
    double current_dist;

	for(int i=0; i < observations.size(); i++){
        double max_dist = 1000;
		for(int j=0; j < predicted.size(); j++){
			current_dist = dist(observations[i].x,  observations[i].y, predicted[j].x, predicted[j].y);
			if(current_dist < max_dist){
				max_dist = current_dist;
				nearest_obs_id = predicted[j].id ;
			}
		}
		observations[i].id = nearest_obs_id;
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
   
   
   double std_x = std_landmark[0];
	double std_y = std_landmark[1];
	double dist_;

	for(int i=0; i < num_particles; i++){
		for(int j=0; j < map_landmarks.landmark_list.size(); j++){
			dist_ = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);

			if( dist_ <= sensor_range ) {
				LandmarkObs l_;
				l_.id = map_landmarks.landmark_list[j].id_i;
				l_.x = map_landmarks.landmark_list[j].x_f;
				l_.y = map_landmarks.landmark_list[j].y_f;
				marks.push_back(l_);
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
