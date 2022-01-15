#ifndef CORRIDOR_H
#define CORRIDOR_H

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "KDTree.hpp"

using namespace std;

namespace AstarOsqp{

class corridor{
public:
        
    void initCollsion(KDTree tree, double cols, double rows);
    
    void update(const vector<pair<double, double>> traj);

    void test();

    vector<vector<double>> boxes;
    vector<vector<double>> vis_boxes;

private:

    bool isBoxInBoundary(const vector<double>& box);

    bool isPointInBox(const pair<double,double> point, const vector<double>& box);

    bool isObstacleInBox(vector<double>& box, double margin);

    void expandBox(vector<double>& box, double margin);

    void updateObsBox();
   
   
    vector<pair<double, double>> traj_;
    KDTree obstacleTree_;

    double cols_;
    double rows_;
};    
}

#endif