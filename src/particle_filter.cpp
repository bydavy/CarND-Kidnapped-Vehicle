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
  // Cache some constants into stack
  double cov_x = std_landmark[0];
  double cov_y = std_landmark[1];
  double cov_x_sq_2 = 2*cov_x*cov_x;
  double cov_y_sq_2 = 2*cov_y*cov_y;

  for (auto & p: particles) {
    // Clear landmarks associated to observations
    for (auto & obs: observations) {
      obs.id = 0;
    }

    // Convert observations from VEHICULE to MAP's coordinate
    std::vector<LandmarkObs> map_observations;
    for (auto const& obs: observations) {
      double cos_t = cos(p.theta);
      double sin_t = sin(p.theta);
      LandmarkObs map_obs = LandmarkObs();
      map_obs.x = p.x + (obs.x * cos_t - obs.y * sin_t);
      map_obs.y = p.y + (obs.x * sin_t + obs.y * cos_t);

      map_observations.push_back(map_obs);
    }

    // Optimization - Filter out landmarks that are out of sensor range for current particle
    std::vector<LandmarkObs> map_landmarks_in_range;
    for (auto const& landmark_s: map_landmarks.landmark_list) {
        double distance = dist(p.x, p.y, landmark_s.x_f, landmark_s.y_f);
        if (distance > sensor_range) {
          // Filter out
          continue;
        }

        LandmarkObs landmark = LandmarkObs();
        landmark.id = landmark_s.id_i;
        landmark.x = landmark_s.x_f;
        landmark.y = landmark_s.y_f;
        map_landmarks_in_range.push_back(landmark);
    }

    // Attempt to associate a landmark to each observation
    dataAssociation(map_landmarks_in_range, map_observations);

    // Update weight
    double prob = 1;
    for (auto const& obs: map_observations) {
      if (obs.id == 0) {
        // Not found a landmark for this observation, skipping
        continue;
      }

      int landmark_index = obs.id - 1;
      Map::single_landmark_s landmark = map_landmarks.landmark_list[landmark_index];

      double x_diff = obs.x - landmark.x_f;
      double y_diff = obs.y - landmark.y_f;
      double e = -( (x_diff*x_diff/cov_x_sq_2) + (y_diff*y_diff/cov_y_sq_2) );
      prob *= (1/(2.*M_PI*cov_x*cov_y)) * exp(e);
    }

    // Set non normalized weight
    p.weight = prob;
  }
}

void ParticleFilter::resample() {
  // Normalize weights
  double sum_weigths = 0;
  for(auto &p : particles) {
    sum_weigths += p.weight;
  }

  weights.clear();
  for(auto &p : particles) {
    double w = p.weight/sum_weigths;
    weights.push_back(w);
  }

  // Start resampling
  std::vector<Particle> particles_resampled;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> d(weights.begin(), weights.end());
  for(int n = 0; n < num_particles; ++n) {
    particles_resampled.push_back(particles[d(gen)]);
  }
  particles = particles_resampled;
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
