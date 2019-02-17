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
std::default_random_engine gen;

//initialize particles with the data from GPS 
void ParticleFilter::init(double x, double y, double theta, double std[]) {
  	
	if(is_initialized)
		return;
	
	num_particles = 200;
	is_initialized = true;
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

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

// add noise and predict prediction the location using control input (yaw rate & velocity) for all particles.
void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {

  	double std_x, std_y, std_theta;
	std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = std_pos[2];
	std::normal_distribution<double> dist_x(0, std_x);
	std::normal_distribution<double> dist_y(0, std_y);
	std::normal_distribution<double> dist_theta(0, std_theta);

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

//find the nearest neighboring particle
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
 	for(int i=0; i < observations.size(); ++i) {
		double min_dist = std::numeric_limits<double>::infinity();
		for(int j=0; j < predicted.size(); ++j) {
			LandmarkObs& predicted_Object = predicted.at(j);
			double eu_dist = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
			if (eu_dist < min_dist) {
				min_dist = eu_dist;
				observations[i].id = predicted[j].id;
			}
		}
	}
}

//update the corresponding weights to each particle
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
   weights.clear();
   for (int i = 0; i < num_particles; ++i) {
	   vector<LandmarkObs> landmarks_MapCord;
		//load values from landmark list
		for (auto& map_landmark : map_landmarks.landmark_list) {
			double distance = dist(particles[i].x, particles[i].y, map_landmark.x_f, map_landmark.y_f);
			if(distance <= sensor_range) {
				LandmarkObs landmark_Object;
				landmark_Object.id = map_landmark.id_i;
				landmark_Object.x = map_landmark.x_f;
				landmark_Object.y = map_landmark.y_f;
				landmarks_MapCord.push_back(landmark_Object);
			}
		}
		vector<LandmarkObs> observations_MapCord;
     		//transform values from sensor coordinates to map coordinates
		  for (auto& observation_sensor : observations) {
			LandmarkObs transformed_landmark_Object;
			transformed_landmark_Object.id = observation_sensor.id;
			transformed_landmark_Object.x = (particles[i].x) + (observation_sensor.x * cos(particles[i].theta)) - (observation_sensor.y * sin(particles[i].theta));
			transformed_landmark_Object.y = (particles[i].y) + (observation_sensor.x * sin(particles[i].theta)) + (observation_sensor.y * cos(particles[i].theta));
			observations_MapCord.push_back(transformed_landmark_Object);
		}
     
	dataAssociation(landmarks_MapCord, observations_MapCord);
     	bool matched = false;
    	std::vector<int> associations;
    	std::vector<double> sense_x;
    	std::vector<double> sense_y;
     	for(auto& observation_Object : observations_MapCord){
                auto correspondency = std::find_if(landmarks_MapCord.begin(), landmarks_MapCord.end(), [observation_Object](LandmarkObs const& landmarks_object){ return landmarks_object.id == observation_Object.id; });
        	if(correspondency != landmarks_MapCord.end()) {
              		double gauss_norm= 1/(2 * M_PI * std_landmark[0] * std_landmark[1]);
            		if(!matched) {
				double exponent = pow(observation_Object.x - correspondency->x, 2)/(2 * pow(std_landmark[0], 2)) + pow(observation_Object.y - correspondency->y, 2)/(2 * pow(std_landmark[1], 2));
                		particles[i].weight = gauss_norm * exp(-exponent);
                		matched = true;
            		} else{
                  		double exponent = pow(observation_Object.x - correspondency->x, 2)/(2 * pow(std_landmark[0], 2)) + pow(observation_Object.y - correspondency->y, 2)/(2 * pow(std_landmark[1], 2));
                		particles[i].weight *= gauss_norm * exp(-exponent); 
            		}
            		auto mapping = std::find_if(map_landmarks.landmark_list.begin(), map_landmarks.landmark_list.end(), [correspondency](Map::single_landmark_s const& single_landmark_object){ return single_landmark_object.id_i == correspondency->id; });

            		associations.push_back(mapping->id_i);
            		sense_x.push_back(mapping->x_f);
            		sense_y.push_back(mapping->y_f);
        	}
        }
	weights.push_back(particles[i].weight);
    	SetAssociations(particles[i],associations,sense_x,sense_y);
    }
}

//Resample the particles to reject the unwanted correspondency
void ParticleFilter::resample() {

   std::random_device rd;
   std::mt19937 gen(rd());
   std::discrete_distribution<> d(weights.begin(),weights.end());
   //double total_weight = std::accumulate(weights.begin(),weights.end(),0.0);
   std::vector<Particle> resampled_particles;
   for (int i = 0; i < num_particles; ++i) {
       Particle resampled_part = particles[d(gen)];
       resampled_particles.push_back(resampled_part);
   }
   particles.clear();
   particles = resampled_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
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
