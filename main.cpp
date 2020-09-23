#include <iostream>
#include <windows.h>
#include <fstream>
#include <vector>
#include <thread>
#include "GetDirection.hpp"
#include "GetDistance.hpp"
#include "TakeStep.hpp"
#include "InCollision.hpp"
#include "rect_obstacle.hpp"
#include "logger.hpp"
#include "TraceObstacle.hpp"

//#include "shape.hpp"
#include "obstacle.hpp"


using namespace std;


int main(int argc, char **argv) {




	// Initialize logging
	// TODO Make this a class/function
	std::ofstream log_file("motion_planning.csv");
	log_file << "X" << "," << "Y" << "\n";


	// Initialize the Workspace
	// Define q_start and q_goal
	std::vector<double> q_start(2);
	q_start[0] = 0;
	q_start[1] = 0;

	std::vector<double> q_goal(2);
	q_goal[0] = 10;
	q_goal[1] = 10;

	// Obstacles
	AlignedRectangle WO1;
	AlignedRectangle WO2;
	AlignedRectangle WO3;
	AlignedRectangle WO4;
	AlignedRectangle WO5;


	WO1.DefineBoundary(1.0, 2.0, 1.0, 5.0);
	WO2.DefineBoundary(3.0, 4.0, 4.0, 12.0);
	WO3.DefineBoundary(3.0, 12.0, 12.0, 13.0);
	WO4.DefineBoundary(12.0, 13.0, 5.0, 13.0);
	WO5.DefineBoundary(6.0, 12.0, 5.0, 6.0);


	// Vector of obstacles
	std::vector<AlignedRectangle> obstacle_vector;
	obstacle_vector.push_back(WO1);
	obstacle_vector.push_back(WO2);
	obstacle_vector.push_back(WO3);
	obstacle_vector.push_back(WO4);
	obstacle_vector.push_back(WO5);



	// Initialize the path and related variables
	std::vector<double> dir_vec(q_start.size());	// Unit vector pointing in direction of motion
	std::vector<double> curr_pos(q_start.size()); 	// the current position of the robot
	std::vector<double> prev_pos(q_start.size());	// The previous position of the robot (used for accumulating distance)
	std::vector<double> leave_pos(q_start.size());	// The position corresponding to closest distance to goal while following obstacle (BUG 1)
	std::vector<double> hit_pos(q_start.size());	// The position corresponding to collision with an obstacle
//	double d_followed;								// Distance from robot to goal while following obstacle (BUG 1)
//	double shortest_dist_to_goal = 0;				// Previous distance from robot to goal while following obstacle (BUG 1)
//	bool atVertex = false;							// Flag indicating if we're at a vertex (BUG 1)
//	double dist_to_vert;							// Distance from robot to vertex (BUG 1)
	double dist_to_leave_pt;						// Distance from leave point for when we're traveling to the leave point (BUG 1)
	AlignedRectangle obs_in_path;					// The obstacle currently in our way
	bool GoalAchieved = false;						// Flag indicating if goal has been achieved
	double dist_to_goal;							// Distance from the robot to the goal
	double step_size = 0.001;						// Step size to take during each loop
	// double obstacle_offset = 2*step_size;		// Distance from obstacle boundary to consider robot "in collision"
	double total_distance_traveled = 0;				// Total distance traveled by the robot
	bool atLeavePoint = false;						// Flag indicating if we're at the leave point of an obstacle
	int mode =  0; 									// O for move to goal, 1 for follow boundary



	// Main algorithm loop
	while (not GoalAchieved)
	{

		if(mode == 0) // Move to goal
		{

			// Get the direction to the goal
			GetDirection(dir_vec, curr_pos, q_goal);

			// Take a step towards the goal and update our current position
			TakeStep(curr_pos, dir_vec, step_size);

			// Check if the point is in collision with any of the obstacles
			for (int i=0; i<int(obstacle_vector.size()); i++)
			{
				//if (InCollision(curr_pos, obstacle_vector[i], obstacle_offset))
				if (obstacle_vector[i].InCollision(curr_pos))
				{
					std::cout<< "COLLISION PT: " << std::endl;
					std::cout<< curr_pos[0] << " " << curr_pos[1] << std::endl;

					// Flag indicating if we're at the leave point (BUG 1)
					atLeavePoint = false;

					// This is the obstacle in our way
					obs_in_path = AlignedRectangle(obstacle_vector[i]);

					// set the hit position of the boundary
					hit_pos = curr_pos;

					// Set the distance to goal
					//shortest_dist_to_goal = GetDistance(hit_pos, q_goal);

					// Change algorithm mode
					mode = 1;

					// Take this obstacle out of our obstacle vector because we're done with this one
					obstacle_vector.erase(obstacle_vector.begin()+i);

				} // End if

			} // End for

			// Position is good so log it
			//bug1_logger.log_data();
			log_file << curr_pos[0] << "," << curr_pos[1] << "\n";

		} // End mode == 0

		else if (mode == 1) // Follow obstacle
		{


			std::vector<vector<double>> path_taken; // = TraceObstacle(curr_pos, obs_in_path, log_file, step_size, q_goal);
			TraceObstacle(curr_pos, obs_in_path, path_taken, leave_pos, log_file, step_size, q_goal); // This can probably go in collision check in mode 1
//			std::vector<vector<double>> path_taken;	// Vector containing all position values the robot takes while following an obstacle (BUG 1)
//
//
//			// ---------ALL THIS SHOULD BE IN A BUG1 FUNCTION-----------------------------------------
//			// Left turning robot determines the order of this vector
//			std::vector<std::vector<double>> vertex_vec = {obs_in_path.v4, obs_in_path.v3, obs_in_path.v2, obs_in_path.v1};
//
//			// add our starting point to the end of vertex vector (locations will be mini goals during boundary follow mode)
//			vertex_vec.push_back(curr_pos);
//			// std::cout<< "VERTEX VEC SIZE: " << vertex_vec.size() << std::endl;
//
//			//std::cout << "vertex vec " << vertex_vec[0][0] << " " << vertex_vec[0][1] <<  std::endl;
//
//
//			// Iterate over each vertex and use those in a mini "move to goal" mode
//			// This will ensure we go all the way around the obstacle
//			for (int i = 0; i < int(vertex_vec.size()); i++)
//			{
//				//std::cout<< "IN FOR LOOP" << std::endl;
//
//				// Move toward the current vertex
//				while (not atVertex)
//				{
//					// std::cout<< "IN WHILE" << std::endl;
//					//std::cout << "vertex vec " << vertex_vec.at(0)[0] << " " << vertex_vec.at(0)[1] <<  std::endl;
//
//					// Get direction to the vertex we're currently targeting
//					GetDirection(dir_vec, curr_pos, std::vector<double>(vertex_vec[i]));
//					//std::cout<< "GOT DIRECTION" << std::endl;
//
//					// Take a step towards the vertex and update our current position
//					TakeStep(curr_pos, dir_vec, step_size);
//					//bug1_logger.log_data();
//					log_file << curr_pos[0] << "," << curr_pos[1] << "\n";
//
//
//					// Add the current position to our memory of the route we've taken so far
//					path_taken.push_back(curr_pos);
//					//std::cout << "vector size " << path_taken.size() << std::endl;
//					//std::cout << "vector [0] " << path_taken[0][1] <<  std::endl;
//
//					d_followed = GetDistance(curr_pos, q_goal);
//
//					// If we find a point on the obstacle that's closer to the goal
//					if (d_followed < shortest_dist_to_goal)
//					{
//						leave_pos = curr_pos;
//						// Update the new closest distance
//						shortest_dist_to_goal = d_followed;
//					}
//
//
//					// Check if we're at the vertex yet
//					dist_to_vert = GetDistance(curr_pos, vertex_vec[i]);
//					if (dist_to_vert < 1e-3)
//					{
//						// std::cout<< "AT VERTEX" << std::endl;
//						atVertex = true;
//					}
//
//				} // End while
//
//				// reset the flag
//				atVertex = false;
//
//			} // End for





//			std::cout<< "LEAVE POS" << std::endl;
//			std::cout<< leave_pos[0] << " " << leave_pos[1] << std::endl;
			while(not atLeavePoint)
			{

				// Use our memory of the route to get to the leave point
				for (int i = 0; i < int(path_taken.size()); i++)
				{
					curr_pos = path_taken[i];
					log_file << curr_pos[0] << "," << curr_pos[1] << "\n";
					// bug1_logger.log_data();

					// Check if we're in collision with a new obstacle
					for (int i=0; i<int(obstacle_vector.size()); i++)
					{
						//if (InCollision(curr_pos, obstacle_vector[i], obstacle_offset))
						if (obstacle_vector[i].InCollision(curr_pos))
						{
							std::cout<< "COLLISION PT: " << std::endl;
							std::cout<< curr_pos[0] << " " << curr_pos[1] << std::endl;

							// Flag indicating if we're at the leave point (BUG 1)
							atLeavePoint = false;

							// This is the obstacle in our way
							obs_in_path = AlignedRectangle(obstacle_vector[i]);

							// Trace the obstacle and update the leave point
							TraceObstacle(curr_pos, obs_in_path, path_taken, leave_pos, log_file, step_size, q_goal);
						}
					}

					// Check how far we are from the leave point
					dist_to_leave_pt = GetDistance(curr_pos, leave_pos);


					if (dist_to_leave_pt < 1.0e-3)
					{
						std::cout<< "AT LEAVE POINT" << std::endl;
						std::cout<< curr_pos[0] << " " << curr_pos[1] << std::endl;
						// We can change back to Move To Goal mode
						atLeavePoint = true;
						mode = 0;

						// reset path_taken
						break;
					}
				}

			} // End while

			// ---------ALL THIS SHOULD BE IN A BUG1 FUNCTION-----------------------------------------

		} // End mode == 1


		else // Something went wrong if we're here
		{
			std::cout<< "ERROR: UNRECOGNIZED MODE" << std::endl;
			break;
		}


		// Check how far we are from the goal
		dist_to_goal = GetDistance(curr_pos, q_goal);

		// Check if we've made it to the goal
		if (abs(dist_to_goal) <= 1e-3)
		{
			// We made it to the goal
			std::cout<< "=== GOAL ACHIEVED! ==="<< std::endl;
			GoalAchieved = true;
		}

		// Increment the total distance traveled and update the previous position
		// put this in logger class
		total_distance_traveled += GetDistance(prev_pos, curr_pos);
		prev_pos = curr_pos;


		std::cout<< "++END OF MAIN WHILE LOOP++" << std::endl;

	} // End while


	std::cout<< "EXITING" << std::endl;
	log_file.close();
	return 0;
}
