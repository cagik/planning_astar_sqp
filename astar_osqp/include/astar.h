/*
 * @Author: cagik
 * @Date: 2021-12-09 14:03:41
 * @LastEditTime: 2022-01-18 13:32:49
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

//Node的类别
enum NodeType{
    obstacle = 0,
    free,
    inOpenList,
    inCloseList
};

//Node astar寻路中的离散量
struct Node{
    
    //储存位置信息
    cv::Point point;

    // F = G + H； G：从起始点到当前Node的路径cost， H：从当前Node到终点的cost花费，由欧式距离计算
    int F, G, H; 

    //父节点，用于路径反转
    Node* parent; 

    //构造函数
    Node(cv::Point _point = cv::Point(0, 0)):point(_point), F(0), G(0), H(0), parent(NULL)
    {
    }
};

//比较两个节点的h值，用于初始化优先级队列openlist，其中具有最小H的Node会被自动排列至队首
struct compare
{
    bool operator() (pair<int, cv::Point> a, pair<int, cv::Point> b) // Comparison function for priority queue
    {
        return a.first > b.first; // min h
    }
};


class Astar{

public:
    //初始化astar
    void InitAstar(cv::Mat& _Map, cv::Mat& Mask);

    //astar寻路
    vector<pair<double, double>> pathPlanning(cv::Point _startPoint, cv::Point _targetPoint);
    
    //将点的坐标转为索引值
    inline int point2index(cv::Point point) {return point.y * Map.cols + point.x;}

    //将点的索引值转为坐标
    inline cv::Point index2point(int index) {return cv::Point(int(index / Map.cols), index % Map.cols);}
    
    //得到障碍物列表
    vector<cv::Point> getObstacle(){return obstaclePoint;};

private:
    //初始化地图
    void MapProcess(cv::Mat& Mask);

    //障碍物边界膨胀
    void configureMap();

    //寻路
    Node* FindPath();

    //找到终点时，反转路径
    void GetPath(Node* TailNode, vector<pair<double,double>>& path);

    //计算两点间距离
    double distance(cv::Point p1, cv::Point p2);

private:

    //与地图相关参数
    cv::Mat Map;
    cv::Mat LabelMap;
    int OccupyThresh;
    
    //起始点信息
    cv::Point startPoint, targetPoint;
    cv::Mat neighbor;

    //astar得到的路径
    vector<Node*> PathList;

    //openlist 储存待访问的点，具体作用可以查询astar算法
    priority_queue<pair<int, cv::Point>, vector<pair<int, cv::Point>>, compare> OpenList;

    //openlist的散列表，可以理解为一个哈希表，储存Node和其对应的index关系
    unordered_map<int, Node*> OpenDict;
    
    //读取地图信息时获取的障碍物列表
    vector<cv::Point> obstaclePoint;
};
}

#endif