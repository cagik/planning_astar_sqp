#ifndef TOOL_H
#define TOOL_H

#include <iostream>
#include <cmath>
#include <vector>
#include <cassert>
#include <ctime>
#include <cfloat>
#include <opencv2/opencv.hpp>

#include "data_struct.h"

namespace AstarOsqp{

class State;
double distance(const State &p1, const State &p2);
double distance(cv::Point p1, cv::Point p2);
double distance(std::pair<double, double> p1, std::pair<double, double> p2);
}
#endif