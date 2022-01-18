/*
 * @Author: cagik
 * @Date: 2022-01-10 21:18:59
 * @LastEditTime: 2022-01-18 13:26:53
 * @LastEditors: cagik
 * @Description: 产生安全走廊
 * @FilePath: /astarOsqp_ws/src/astar_osqp/include/corridor.h
 * cavata_iwnl
 */
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
    
    //传入障碍物信息
    void initCorridor(KDTree tree, double cols, double rows);
    
    //得到安全走廊的外部接口
    void update(const vector<pair<double, double>> traj);

    //之前用于测试kdtree的一个函数，现已无用
    void test();

    //所有的box集合
    vector<vector<double>> boxes;

    //用于可视化的box集合
    vector<vector<double>> vis_boxes;

private:

    //检测box是否超出地图边界
    bool isBoxInBoundary(const vector<double>& box);

    //检测当前点是否在box内
    bool isPointInBox(const pair<double,double> point, const vector<double>& box);

    //检测障碍物是否在box内
    bool isObstacleInBox(vector<double>& box, double margin);

    //延展box
    void expandBox(vector<double>& box, double margin);

    //产生box的私有函数
    void updateObsBox();
   
    //需要产生安全走廊的轨迹
    vector<pair<double, double>> traj_;
    
    //障碍物的kdtree，用于获得最近障碍物
    KDTree obstacleTree_;

    //地图边界信息
    double cols_;
    double rows_;
};    
}

#endif