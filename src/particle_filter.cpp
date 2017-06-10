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
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	if (!is_initialized_){
	num_particles = 1000;
	double std_x, std_y, std_psi; // Standard deviations for x, y, and psi
	// Set standard deviations for x, y, and psi
	 std_x = std[0];
	 std_y = std[1];
	 std_psi = std[2];
	 
	 //Initilaize a random engine names "gen"
	 std::default_random_engine gen;
	 
	 // creates a normal (Gaussian) distribution for x,y and theta
	std::normal_distribution<double> dist_x(x, std_x);
	std::normal_distribution<double> dist_y(y, std_y);
	std::normal_distribution<double> dist_psi(theta, std_psi);

	//weights = std::fill(weights.begin(), v.end(), 1);
	//weights = std::fill(0, num_particles, 1.0);
	weights.resize(num_particles,1);//set the weights to 1
	
	//Initialize all the particles to the (first)GPS position as mean
	for (i = 0; i < num_particles; ++i){
		particles[i].id = i+1;
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta  = dist_psi(gen);
		particles[i].weight = weights[i];
	}
	is_initialized_ = true;
	}
	

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	//Initialize all the particles to the (first)GPS position as mean
	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_psi = std_pos[2];
	//Initilaize a random engine names "gen"
	std::default_random_engine genP;
	
	for (i = 0; i < num_particles; ++i){
		
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;
		//Add measurements using the physical Model
		if (fabs(yaw_rate) < 0.00001){ //Yawrate zero case Handle
			x += velocity * delta_t*cos(theta);
			y += velocity * delta_t*sin(theta);
		}
		else{
			
			x += velocity/yaw_rate * (sin(theta + (yaw_rate*delta_t)) - sin(theta));
		    y += velocity/yaw_rate * (cos(theta) - cos(theta+(yaw_rate*delta_t)));
		    theta += yaw_rate*delta_t;
		}
				
		 // Add Noise - creates a normal (Gaussian) distribution for x,y and theta
	    std::normal_distribution<double> noise_x(0, std_x);
	    std::normal_distribution<double> noise_y(0, std_y);
	    std::normal_distribution<double> noise_psi(0, std_psi);
		
		particles[i].x = x + noise_x(genP);
		particles[i].y = y + noise_y(genP);
		particles[i].theta  = theta + noise_psi(genP);
		
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (int i =0; i < observations.size();i++){
		double min_length = 1000;
		int index = -1;
		for (int j =0; j < predicted.size(); j++){
			double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
			if (distance < min_length){
				min_length = distance;
				index = j;
			}
		}
		//Found the smallest distance. Now asssign the values of the prediction to the observation
		observations[i].id = predicted[index].id;
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
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	std::vector<LandmarkObs> tObservations; //Transformed Observations Vector
	std::vector<LandmarkObs> filtered_landmarks; //Hold Landmarks in sensor Range
	double s_x = std_landmark[0]; std deviation of the meas in x
	double s_y = std_landmark[0]; std deviation of the meas in y
	double s_theta = std_landmark[1]; std deviation of the meas in theta
	//x*cos(theta) - y*sin(theta)+ xt
	//x*sin(theta) + y*cos(theta) + yt
	for (i = 0; i < num_particles; ++i){
		
		double p_x = particles[i].x;
		double p_y = particles[i].y;
		double p_theta = particles[i].theta;
		
		tObservations.clear(); // reset Transformed Observations Vector[]
		LandmarkObs t_obs; //  Transformed Observation for Single particle
		
		for (int j= 0 ; j < observations.size();j+){
			t_obs.x = cos(p_theta)*observations[j].x - sin(theta)*observations[j].y + x;
			t_obs.y = sin(p_theta)*observations[j].x + cos(theta)*observations[j].y + y;
			t_obs.id = observations[j].id;
			tObservations.push_back(t_obs);
		}
				
		// Filter Valid Predictions based on sensor Range
		filtered_landmarks.clear();
		for ( int k=0; k< map_landmarks.size();++k){
			if (dist(tObservations[j].x,map_landmarks[k].x_f,tObservations[j].y,map_landmarks[k].y_f) < sensor_range){
			filtered_landmarks.push_back(single_landmark_s{map_landmarks[k].id_i,map_landmarks[k].x_f,map_landmarks[k].y_f,})}
		}
				
		//Find the prediction that points to a Observation, finds the probability/Weights
		dataAssociation(filtered_landmarks,tObservations);
		double total_prob = 1.0;
		
		for (int m =0; m<tObservations.size();m++){
			for ( int idx = 0 ; idx<  filtered_landmarks.size();idx++){
				if (tObservations[m].id == filtered_landmarks[idx].id){
					//Find the nearest landmark for the observation and use that landmark instead of the observation
					double pr_x = filtered_landmarks[idx].x;
					double pr_y = filtered_landmarks[idx].y;
					//break;
				}
			}
			// Calculate Multivariate-Gaussian Probability for each observations(mesurements)
			auto d_x = pr_x - p_x;
			auto d_y = pr_y - p_y;
			//Total probability is the product of individual measurement Probabilities. 
			total_prob *= (1/(2*M_PI*s_x*s_y))* exp -(((d_x*d_x)/(2*s_x*s_x)) + ((d_y*d_y)/(2*s_y*s_y)));
		}
		//Update the weights
		particles[i].weight = total_prob;
		weights[i] = total_prob;
	}							
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	std::discrete_distribution<int> d(weights.begin(), weights.end()); // Define a discrete distribution
	std::vector<Particle> new_particles; // Resampled particles holder
	std::default_random_engine gen3
	
	for (int i = 0; i< num_particles; i++){
		auto index = d(gen3);
		new_particles.push_back(std::move(particles[index]));
	}
	assign the particles from folder the original
	particles = std::move(new_particles);

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
