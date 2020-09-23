#ifndef OBSTACLE_HPP_
#define OBSTACLE_HPP_
#include "shape.hpp"
#include <iostream>
#include <vector>
using namespace std;



class AlignedRectangle : public Shape {


    public:
        double min_x;
        double max_x;
        double min_y;
        double max_y;
        std::vector<double> v4;
        std::vector<double> v3;
        std::vector<double> v2;
        std::vector<double> v1;
        //std::vector<std::vector<double>> vertex_vector;

        AlignedRectangle();
        //AlignedRectangle(double x_left, double x_right, double y_bottom, double y_top);
        AlignedRectangle(std::vector<std::vector<double>> vertex_vector);


        ~AlignedRectangle();

        void DefineBoundary(double x_left, double x_right, double y_bottom, double y_top);
        //void DefineBoundary(std::vector<std::vector<double>> vertex_vector);

        bool InCollision(std::vector<double> pt_to_check);

        //void ApplyBoundaryOffset(double offset);



};


#endif
