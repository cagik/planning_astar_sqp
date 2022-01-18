/*
 * @Author: cagik
 * @Date: 2022-01-10 21:18:59
 * @LastEditTime: 2022-01-18 13:39:00
 * @LastEditors: cagik
 * @Description: 放了三个函数，我承认这个h文件很丑陋
 * @FilePath: /astarOsqp_ws/src/astar_osqp/include/tool.h
 * cavata_iwnl
 */
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
//
double distance(const State &p1, const State &p2);
double distance(cv::Point p1, cv::Point p2);
double distance(std::pair<double, double> p1, std::pair<double, double> p2);
}
#endif