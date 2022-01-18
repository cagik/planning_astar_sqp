#include "tool.h"

namespace AstarOsqp{

double distance(const State &p1, const State &p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double distance(cv::Point p1, cv::Point p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double distance(std::pair<double, double> p1, std::pair<double, double> p2){
    return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second- p2.second, 2));
}
}