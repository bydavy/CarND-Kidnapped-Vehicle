/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

#define ZERO_YAW_RATE_THRESHOLD 0.00001

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  std::default_random_engine gen;

	// This creates a normal (Gaussian) distribution for x, y and theta
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

  num_particles = 1000; // TODO Rework number of particles
	for (int i = 0; i < num_particles; ++i) {
		Particle p = Particle();
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1;
		particles.push_back(p);
    // TODO what is the diff between p.weight and weights
    weights.push_back(1);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  std::default_random_engine gen;

  for(auto &p: particles) {
    if (yaw_rate < ZERO_YAW_RATE_THRESHOLD && yaw_rate > -ZERO_YAW_RATE_THRESHOLD) {
      p.x += velocity * delta_t * cos(p.theta);
      p.y += velocity * delta_t * sin(p.theta);
      // Theta is unchanged as yaw_rate is null
    } else {
      double c1 = velocity / yaw_rate;
      double new_yaw = p.theta + yaw_rate * delta_t;
      p.x += c1 * (sin(new_yaw) - sin(p.theta));
      p.y += c1 * (cos(p.theta) - cos(new_yaw));
      p.theta = new_yaw;
    }

    // This creates a normal (Gaussian) distribution for x, y and theta
    std::normal_distribution<double> dist_x(p.x, std_pos[0]);
    std::normal_distribution<double> dist_y(p.y, std_pos[1]);
    std::normal_distribution<double> dist_theta(p.theta, std_pos[2]);
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // TODO Seriously the prototype of this method is terrible!!! It doesn't allow the case where no landmark correspond to an observation
  int obs_size = observations.size();
  for (int i = 0; i < obs_size; ++i) {
    LandmarkObs obs = observations[i];
    LandmarkObs closest = predicted[0]; // FIXME out of bounds
    double closest_distance = dist(obs.x, obs.y, closest.x, closest.y);
    for(auto const& p: predicted) {
      double distance = dist(obs.x, obs.y, p.x, p.y);
      if (distance < closest_distance) { // New closest found
        closest = p;
        closest_distance = distance;
      }
    }
    observations[i] = closest;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
  for (auto & p: particles) {
    // Convert observations from VEHICULE to MAP's coordinate
    std::vector<LandmarkObs> map_observations;
    for (auto const& obs: observations) {
      LandmarkObs map_obs = LandmarkObs();
      map_obs.id = obs.id; // Copy over the id
      map_obs.x = p.x * cos(p.theta) - p.y * sin(p.theta) + obs.x;
      // Map's y points downards, therefore we've a minus
      map_obs.y = -(p.x * sin(p.theta) + p.y * cos(p.theta)) + obs.y;

      map_observations.push_back(map_obs);
    }

    // Keep only map landmarks that are in sensor range
    std::vector<LandmarkObs> map_landmarks_in_range;
    for (auto const& landmark_s: map_landmarks.landmark_list) {
        double distance = dist(p.x, p.y, landmark_s.x_f, landmark_s.y_f);
        if (distance > sensor_range) {
          // Filter out landmarks out of range
          continue;
        }

        LandmarkObs landmark = LandmarkObs();
        landmark.id = landmark_s.id_i;
        landmark.x = landmark_s.x_f;
        landmark.y = landmark_s.y_f;
        map_landmarks_in_range.push_back(landmark);
    }

    // Associate a landmark to each observation
    dataAssociation(map_landmarks_in_range, map_observations);

    // Set weight
    // TODO Finish weight processing
    p.weight = 1;
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
