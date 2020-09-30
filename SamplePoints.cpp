/*
 * SamplePoints.cpp
 *
 *  Created on: Sep 23, 2020
 *      Author: Joe
 */

#include "SamplePoints.hpp"

void SamplePoints(std::vector<double>& current_pos, std::vector<std::vector<double>>& sampled_points)
{

	// Get a vector of points surrounding the current position
	for(int i=0; i<359; i++)
	{
		std::vector<double> sampled_pt(current_pos.size());
		//double theta = i * M_PI/180;
		double theta = i * 3.1415/180;

		//TODO: make the radius of these points equal to the step size, not 1

		sampled_pt[0] = current_pos[0] + cos(theta);
		sampled_pt[1] = current_pos[1] + sin(theta);
		sampled_points.push_back(sampled_pt);

	}

}


