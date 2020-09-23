/*
 * obstacle.cpp
 *
 *  Created on: Sep 21, 2020
 *      Author: Joe
 */


// #include "shape.hpp"
#include "obstacle.hpp"


AlignedRectangle::AlignedRectangle() {


    min_x = 0;
    max_x = 0;
    min_y = 0;
    max_y = 0;
    v4 = {{min_x, max_y}};
    v3 = {{max_x, max_y}};
    v2 = {{max_x, min_y}};
    v1 = {{min_x, min_y}};
	//vertex_vector = {{0, 0}};

}

void AlignedRectangle::DefineBoundary(double x_left, double x_right, double y_bottom, double y_top)
//void AlignedRectangle::DefineBoundary(std::vector<std::vector<double>> vertex_vector)
{
	min_x = x_left;
	max_x = x_right;
	min_y = y_bottom;
	max_y = y_top;

	v4 = {{min_x, max_y}};
    v3 = {{max_x, max_y}};
    v2 = {{max_x, min_y}};
    v1 = {{min_x, min_y}};
}

AlignedRectangle::~AlignedRectangle()
{

}


// Check if given point within the boundary of the rectangle
bool AlignedRectangle::InCollision(std::vector<double> pt_to_check)
{
	bool pt_collides = false;
	if (pt_to_check[0] < max_x && pt_to_check[0] > min_x)
	{
		if (pt_to_check[1] < max_y && pt_to_check[1] > min_y)
		{
			pt_collides = true;
		}
	}
	return pt_collides;
}

//void AlignedRectangle::ApplyBoundaryOffset(double offset)
//        {
//
//        }









