/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"
#include "multiv_gauss.h"

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

  num_particles = 20;  // TODO: Set the number of particles

  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);
  std::default_random_engine gen;
  
  for (int i=0; i< num_particles; i++){
    Particle p;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;
    particles.push_back(p);
  }
  is_initialized = true;
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

  std::default_random_engine gen;
  for (int i=0; i< num_particles; i++){
    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;
    if (fabs(yaw_rate) > 1e-4){
      x = x + velocity/yaw_rate*(+sin(theta+yaw_rate*delta_t)-sin(theta));
      y = y + velocity/yaw_rate*(-cos(theta+yaw_rate*delta_t)+cos(theta));
      theta = theta + yaw_rate*delta_t;
    }
    else {
      x = x + velocity*cos(theta)*delta_t;
      y = y + velocity*sin(theta)*delta_t;
    }
    std::normal_distribution<double> dist_x(x, std_pos[0]);
    std::normal_distribution<double> dist_y(y, std_pos[1]);
    std::normal_distribution<double> dist_theta(theta, std_pos[2]);
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
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

  for (unsigned int i=0; i<observations.size(); i++){
    int closest = -1;
    double min_distance = 1.0e6;
    double distance;
    if (predicted.size() > 0) {
      for (unsigned int j=0; j<predicted.size(); j++){
        distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
        if (distance <= min_distance) {
          closest = j;
          min_distance = distance;
        }
      }
      observations[i].id = predicted[closest].id;
    }
    else
    {
      std::cout << "Association unsuccessful. No predicted in the range." << std::endl;
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

  double xp, yp, thetap, xl, yl, distance, total_weight;
  vector<LandmarkObs> observed_landmarks, predicted_landmarks;
  total_weight = 0.0;
  for ( int i=0; i<num_particles; i++){
    xp = particles[i].x;
    yp = particles[i].y;
    thetap = particles[i].theta;
    observed_landmarks.clear();
    for (unsigned int j=0; j<observations.size(); j++){
      xl = xp + observations[j].x*cos(thetap)-observations[j].y*sin(thetap);
      yl = yp + observations[j].x*sin(thetap)+observations[j].y*cos(thetap);
      LandmarkObs map_observation;
      map_observation.id = -1;
      map_observation.x  = xl;
      map_observation.y  = yl;
      observed_landmarks.push_back(map_observation);
    }
    predicted_landmarks.clear();
    for (unsigned int j=0; j<map_landmarks.landmark_list.size(); j++) {
      distance = dist(xp, yp, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);
      if (distance <= sensor_range) {
        LandmarkObs predicted_landmark;
        predicted_landmark.id = map_landmarks.landmark_list[j].id_i;
        predicted_landmark.x = map_landmarks.landmark_list[j].x_f;
        predicted_landmark.y = map_landmarks.landmark_list[j].y_f;
        predicted_landmarks.push_back(predicted_landmark);
      }  
    }
    dataAssociation(predicted_landmarks, observed_landmarks); 
    particles[i].weight = 1.0;   
    for (unsigned int j=0; j<observed_landmarks.size(); j++){
      bool found = false;
      unsigned int k = 0;
      while (!found && k < predicted_landmarks.size()) {
        if (predicted_landmarks[k].id == observed_landmarks[j].id) {
          particles[i].weight *= multiv_prob(std_landmark[0], std_landmark[1], 
                                observed_landmarks[j].x,  observed_landmarks[j].y,
                                predicted_landmarks[k].x, predicted_landmarks[k].y);
          particles[i].weight = std::max(particles[i].weight, 1.0e-9);
          found = true;
        }
        k++;
      }
    }
    total_weight += particles[i].weight;
  }

  for (int i=0; i< num_particles; i++){
    particles[i].weight = particles[i].weight / total_weight;
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  double max_w = 0.0;
  for (int i=0; i< num_particles; i++){
    max_w = std::max(max_w,particles[i].weight);
  }
  double beta = 0.0;
  int index, N;
  N = particles.size();
  std::vector<Particle> new_particles;
  new_particles.clear();
  for (int i=0; i< num_particles; i++){
    beta = beta + 2.0*max_w;
    index = std::rand() % N;
    while (particles[index].weight < beta){
      beta -= particles[index].weight;
      index = (index +1) % N;particles.size();
    }
    new_particles.push_back(particles[index]);
  }
  particles = new_particles;
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