/*
 * @Author: cagik
 * @Date: 2021-12-09 14:03:41
 * @LastEditTime: 2022-01-05 10:43:06
 * @LastEditors: cagik
 * @Description: 前端astar寻路
 * @FilePath: /astarOsqp_ws/src/astar_osqp/include/astar.h
 * cavata_iwnl
 */
#ifndef ASTAR_H
#define ASTAR_H

#include <queue>
#include <unordered_map>
#include <opencv2/opencv.hpp>


using namespace std;

namespace AstarOsqp{

enum NodeType{
    obstacle = 0,
    free,
    inOpenList,
    inCloseList
};

struct Node{
    cv::Point point;  // node coordinate
    int F, G, H;  // cost
    Node* parent; // parent node

    Node(cv::Point _point = cv::Point(0, 0)):point(_point), F(0), G(0), H(0), parent(NULL)
    {
    }
};

struct compare
{
    bool operator() (pair<int, cv::Point> a, pair<int, cv::Point> b) // Comparison function for priority queue
    {
        return a.first > b.first; // min h
    }
};


class Astar{

public:
    // Interface function
    void InitAstar(cv::Mat& _Map, cv::Mat& Mask);

    vector<pair<double, double>> pathPlanning(cv::Point _startPoint, cv::Point _targetPoint);
    
    inline int point2index(cv::Point point) {
        return point.y * Map.cols + point.x;
    }
    inline cv::Point index2point(int index) {
        return cv::Point(int(index / Map.cols), index % Map.cols);
    }
    
    vector<cv::Point> getObstacle(){return obstaclePoint;};

private:
    //function
    void MapProcess(cv::Mat& Mask);
    void configureMap();
    Node* FindPath();
    void GetPath(Node* TailNode, vector<pair<double,double>>& path);
    double distance(cv::Point p1, cv::Point p2);

private:
    //Object
    cv::Mat Map;
    cv::Point startPoint, targetPoint;
    cv::Mat neighbor;

    priority_queue<pair<int, cv::Point>, vector<pair<int, cv::Point>>, compare> OpenList;
    unordered_map<int, Node*> OpenDict; // open dict
    vector<Node*> PathList;  // path list

    int OccupyThresh;
    cv::Mat LabelMap;

    vector<cv::Point> obstaclePoint;
};
}

#endif