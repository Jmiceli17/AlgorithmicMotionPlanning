/*
 * TraceObstacle.cpp
 *
 *  Created on: Sep 22, 2020
 *      Author: Joe
 *
 *      Function to trace an AlignedRectangle object and return the path to the leave point
 */


#include "TraceObstacle.hpp"


//std::vector<std::vector<double>> TraceObstacle(std::vector<double> &curr_pos, AlignedRectangle &obstacle, std::ofstream &log_file, double step_size, std::vector<double> &q_goal)
void TraceObstacle(std::vector<double> &curr_pos, AlignedRectangle &obstacle, std::vector<std::vector<double>> &path_taken, std::vector<double> &leave_pos, std::ofstream &log_file, double step_size, std::vector<double> &q_goal)

{
	//std::vector<std::vector<double>> path_taken = {curr_pos};  		// Vector containing all position values the robot takes while following an obstacle
	std::vector<double> dir_vec(curr_pos.size());					// Unit vector pointing in direction of motion
	//std::vector<double> leave_pos(curr_pos.size());					// The position corresponding to closest distance to goal while following obstacle (BUG 1)
	bool atVertex = false;											// Flag indicating if we're at a vertex (BUG 1)
	double d_followed; // = 0; 										// Distance from robot to goal while following obstacle (BUG 1)
	double shortest_dist_to_goal = GetDistance(curr_pos, q_goal);	// Previous distance from robot to goal while following obstacle (BUG 1)
	double dist_to_vert = 0;										// Distance from robot to vertex (BUG 1)


	// add our starting point and obstacle vertices to vertex vector (locations will be mini goals during boundary follow mode)
	std::vector<std::vector<double>> vertex_vec = {obstacle.v4, obstacle.v3, obstacle.v2, obstacle.v1};
	vertex_vec.push_back(curr_pos);



	// Iterate over each vertex and use those in a mini "move to goal" mode
	// This will ensure we go all the way around the obstacle
	for (int i = 0; i < int(vertex_vec.size()); i++)
	{

		// Move toward the current vertex
		while (not atVertex)
		{
			// Get direction to the vertex we're currently targeting
			GetDirection(dir_vec, curr_pos, vertex_vec[i]);

			// Take a step towards the vertex and update our current position
			TakeStep(curr_pos, dir_vec, step_size);
			log_file << curr_pos[0] << "," << curr_pos[1] << "\n";

			// Add the current position to our memory of the route we've taken so far
			path_taken.push_back(curr_pos);

			d_followed = GetDistance(curr_pos, q_goal);

			// If we find a point on the obstacle that's closer to the goal
			if (d_followed < shortest_dist_to_goal)
			{
				leave_pos = curr_pos;
				// Update the new closest distance
				shortest_dist_to_goal = d_followed;
			}

			// Check if we're at the vertex yet
			dist_to_vert = GetDistance(curr_pos, vertex_vec[i]);

			if (dist_to_vert < 1e-3)
			{
				// std::cout<< "AT VERTEX" << std::endl;
				atVertex = true;
			}
		} // End while

		// reset the flag
		atVertex = false;

	} // End for

	//return path_taken;
	//return leave_pos;


} // End TraceObstacle


