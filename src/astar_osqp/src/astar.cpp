#include "astar.h"


namespace AstarOsqp{

/**
 * @description: 初始化astar
 * @param {Mat&} _Map
 * @param {Mat&} Mask
 * @return {*}
 */
void Astar::InitAstar(cv::Mat& _Map, cv::Mat& Mask)
{
    char neighbor8[8][2] = {
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},            {0, 1},
            {1, -1},   {1, 0},  {1, 1}
    };

    char bound24[24][2] = {

        {-2, -2},{-2, -1},{-2, 0},{-2, 1},{-2, 2},
        {-1, -2},{-1, -1},{-1, 0},{-1, 1},{-1, 2},
        { 0, -2},{ 0, -1},        { 0, 1},{ 0, 2},
        { 1, -2},{ 1, -1},{ 1, 0},{ 1, 1},{ 1, 2},
        { 2, -2},{ 2, -1},{ 2, 0},{ 2, 1},{ 2, 2}
    };

    Map = _Map;
    neighbor = cv::Mat(24, 2, CV_8S, bound24).clone();
    Astar::OccupyThresh = -1;

    MapProcess(Mask);
    
    configureMap();
}

/**
 * @description: 初始化地图
 * @param {Mat&} Mask
 * @return {*}
 */
void Astar::MapProcess(cv::Mat& Mask)
{
    int width = Map.cols;
    int height = Map.rows;
    cv::Mat _Map = Map.clone();

    // Binarize
    if(OccupyThresh < 0)
    {
        threshold(_Map.clone(), _Map, 0, 255, cv::THRESH_OTSU);
    } else
    {
        threshold(_Map.clone(), _Map, OccupyThresh, 255, cv::THRESH_BINARY);
    }

    
    // Inflate
    cv::Mat src = _Map.clone();

    // Get mask
    cv::bitwise_xor(src, _Map, Mask);

    // Initial LabelMap
    LabelMap = cv::Mat::zeros(height, width, CV_8UC1);
    
    for(int y = 0 ;y<height;y++)
    {
        for(int x = 0;x<width;x++)
        {
            if(_Map.at<uchar>(y, x) == 0)
            {
                LabelMap.at<uchar>(width-1-x, height-1-y) = obstacle;
                cv::Point p;
                p.y = width-1-x;
                p.x = height-1-y;
                obstaclePoint.push_back(p);
            }
            else
            {   
                LabelMap.at<uchar>(width-1-x, height-1-y) = free;
            }
        }
    }
    
}


/**
 * @description: 障碍物简单膨胀
 * @param {*}
 * @return {*}
 */
void Astar::configureMap(){
    for(int i =0; i< obstaclePoint.size(); ++i){
        for(int j =0; j<neighbor.rows; ++j){

            int neighbor_x =  obstaclePoint[i].x + neighbor.at<char>(j,1);
            int neighbor_y =  obstaclePoint[i].y + neighbor.at<char>(j,0);
            if(neighbor_x > 0 && neighbor_x < LabelMap.cols && neighbor_y > 0 && neighbor_y < LabelMap.rows)
            {
                LabelMap.at<uchar>(neighbor_y, neighbor_x) = obstacle;
            }       
        }
    }
}

/**
 * @description: astar寻路，返回一个vector
 * @param {Point} _startPoint
 * @param {Point} _targetPoint
 * @return {vector<cv::Point>}
 */
vector<pair<double,double>> Astar::pathPlanning(cv::Point _startPoint, cv::Point _targetPoint){

    // Get variables
    startPoint = _startPoint;
    targetPoint = _targetPoint;
    vector<pair<double,double>> path;

    // Path Planning
    Node* TailNode = FindPath();
    GetPath(TailNode, path);

    return path;
}

/**
 * @description: 寻路
 * @param {*}
 * @return {*}
 */
Node* Astar::FindPath()
{
    int width = Map.cols;
    int height = Map.rows;

    cv::Mat _LabelMap = LabelMap.clone();

    // Add startPoint to OpenList
    Node* startPointNode = new Node(startPoint);
    OpenList.push(pair<int, cv::Point>(startPointNode->F, startPointNode->point));
    int index = point2index(startPointNode->point);
    OpenDict[index] = startPointNode;
    _LabelMap.at<uchar>(startPoint.y, startPoint.x) = inOpenList;

    while(!OpenList.empty())
    {
        // Find the node with least F value
        cv::Point CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenDict[index];
        OpenDict.erase(index);

        int curX = CurPoint.x;
        int curY = CurPoint.y;
        _LabelMap.at<uchar>(curY, curX) = inCloseList;

        // Determine whether arrive the target point
        if(curX == targetPoint.x && curY == targetPoint.y)
        {
            return CurNode; // Find a valid path
        }

        // Traversal the neighborhood
        for(int k = 0;k < neighbor.rows;k++)
        {   
            int y = curY + neighbor.at<char>(k, 0);
            int x = curX + neighbor.at<char>(k, 1);
            if(_LabelMap.at<uchar>(y, x) == free || _LabelMap.at<uchar>(y, x) == inOpenList)
            {
                
                // Determine whether a diagonal line can pass
                int dist1 = abs(neighbor.at<char>(k, 0)) + abs(neighbor.at<char>(k, 1));
                if(dist1 == 2 && _LabelMap.at<uchar>(y, curX) == obstacle && _LabelMap.at<uchar>(curY, x) == obstacle)
                    continue;

                // Calculate G, H, F value
                int addG, G, H, F;
                if(dist1 == 2)
                {
                    addG = 14;
                }
                else
                {
                    addG = 10;
                }
                G = CurNode->G + addG;
                
                int dist2 = (x - targetPoint.x) * (x - targetPoint.x) + (y - targetPoint.y) * (y - targetPoint.y);
                H = round(10 * sqrt(dist2));

               // H = 10 * (abs(x - targetPoint.x) + abs(y - targetPoint.y));
                F = G + H;

                // Update the G, H, F value of node
                if(_LabelMap.at<uchar>(y, x) == free)
                {
                    Node* node = new Node();
                    node->point = cv::Point(x, y);
                    node->parent = CurNode;
                    node->G = G;
                    node->H = H;
                    node->F = F;
                    OpenList.push(pair<int, cv::Point>(node->F, node->point));
                    int index = point2index(node->point);
                    OpenDict[index] = node;
                    _LabelMap.at<uchar>(y, x) = inOpenList;
                }
                else // _LabelMap.at<uchar>(y, x) == inOpenList
                {
                    // Find the node
                    int index = point2index(cv::Point(x, y));
                    Node* node = OpenDict[index];
                    if(G < node->G)
                    {
                        node->G = G;
                        node->F = F;
                        node->parent = CurNode;
                    }
                }
            }
        }
    }
    cout << "astar failure" << endl;
    return NULL; // Can not find a valid path
}


/**
 * @description: 反转路径
 * @param {Node*} TailNode
 * @return {*}
 */
void Astar::GetPath(Node* TailNode, vector<pair<double,double>>& path)
{
    PathList.clear();
    path.clear();

    // Save path to PathList
    Node* CurNode = TailNode;
    while(CurNode != NULL)
    {
        PathList.push_back(CurNode);
        CurNode = CurNode->parent;
    }

    // Save path to vector<Point>
    int length = PathList.size();
    for(int i = 0;i < length;i++)
    {
        path.push_back(make_pair(PathList.back()->point.x, PathList.back()->point.y));
        PathList.pop_back();
    }

    // Release memory
    while(OpenList.size()) {
        cv::Point CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenDict[index];
        delete CurNode;
    }
    OpenDict.clear();
}

/**
 * @description: 计算两点间欧式距离
 * @param {Point} p1
 * @param {Point} p2
 * @return {*}
 */
double Astar::distance(cv::Point p1, cv::Point p2)
{
    return sqrt(pow(p1.x-p2.x, 2) + pow(p1.y - p2.y, 2));
}

} 