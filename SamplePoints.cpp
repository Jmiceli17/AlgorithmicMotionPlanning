/*
 * SamplePoints.cpp
 *
 *  Created on: Sep 23, 2020
 *      Author: Joe
 */

#include "SamplePoints.hpp"

std::vector<std::vector<double>> SamplePoints(std::vector<double> &current_pos)
{
	std::vector<double> sampled_pt;
	std::vector<std::vector<double>> sampled_points;

	// Get a vector of points surround the current position
	for(int i=0; i<360; i++)
	{
		//double theta = i * M_PI/180;
		double theta = i * 3.1415/180;


		sampled_pt[0] = current_pos[0] + cos(theta);
		sampled_pt[1] = current_pos[1] + sin(theta);
		sampled_points.push_back(sampled_pt);
	}

	return sampled_points;

}


