/*
 * @Author: cagik
 * @Date: 2021-11-30 14:52:30
 * @LastEditTime: 2022-01-05 10:41:35
 * @LastEditors: cagik
 * @Description: testdemo
 * @FilePath: /astarOsqp_ws/src/astar_osqp/src/main.cpp
 * cavata_iwnl
 */
#include <string>
#include <vector>
#include <math.h>  
#include <ctime>

#include <ros/ros.h>
#include <ros/package.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>

#include <ros_viz_tools/ros_viz_tools.h>

#include "astar.h"
#include "data_struct.h"
#include "eigen2cv.hpp"
#include "tool.h"
#include "smoothosqpproblem.h"
#include "interpolation1d.h"
#include "KDTree.hpp"
#include "corridor.h"


#define PI_ 3.1415926

using namespace std;

AstarOsqp::State start_state, end_state;
vector<AstarOsqp::State> ref_point_plot;
bool start_state_rcv = false, end_state_rcv = false, reference_rcv = false;

/**
 * @description: 添加锚点的回调函数 点选至少两个ref_point
 * @param {PointStampedConstPtr} &p 在rviz中点选的publish point
 * @return {*}
 */
void referenceCb(const geometry_msgs::PointStampedConstPtr &p) {
    AstarOsqp::State reference_point;
    reference_point.x = p->point.x;
    reference_point.y = p->point.y;
    ref_point_plot.emplace_back(reference_point);
    if(ref_point_plot.size() >= 2){
        reference_rcv = true;
    }
}

/**
 * @description: 添加start state的回调函数
 * @param {PoseWithCovarianceStampedConstPtr} &start 在rviz中点选的start point，有pose信息
 * @return {*}
 */
void startCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start) {
    start_state.x = start->pose.pose.position.x;
    start_state.y = start->pose.pose.position.y;
    start_state.heading = tf::getYaw(start->pose.pose.orientation);
    start_state_rcv = true;
    std::cout << "get initial state." << std::endl;
}

/**
 * @description: 添加goal state的回调函数
 * @param {PoseStampedConstPtr} &goal 在rviz中点选的goal point，有pose信息
 * @return {*}
 */
void goalCb(const geometry_msgs::PoseStampedConstPtr &goal) {
    end_state.x = goal->pose.position.x;
    end_state.y = goal->pose.position.y;
    end_state.heading = tf::getYaw(goal->pose.orientation);
    end_state_rcv = true;
    std::cout << "get the goal." << std::endl;
}

/**
 * @description: 根据start state产生的引导点，原理为根据start的pose方向产生向前5个size距离的point
 * @param {State} &start_state
 * @return {*}
 */
cv::Point get_StartHeadingPoint(const AstarOsqp::State &s_state){
    cv::Point ref_start_Point;
    ref_start_Point.x = s_state.x + cos(s_state.heading)*10;
    ref_start_Point.y = s_state.y + sin(s_state.heading)*10;
    return ref_start_Point;
}

pair<double, double> get_StartHeadingPoint_double(const AstarOsqp::State& s_state){
    return make_pair(s_state.x + cos(s_state.heading)*5, s_state.y + sin(s_state.heading)*5);
}

/**
 * @description: 与上面的作用类似，不过是产生向后的point
 * @param {State} &goal_state
 * @return {ref_goal_Point}
 */
cv::Point get_GoalHeadingPoint(const AstarOsqp::State &g_state){
    cv::Point ref_goal_Point;
    ref_goal_Point.x = g_state.x - cos(g_state.heading)*10;
    ref_goal_Point.y = g_state.y - sin(g_state.heading)*10;
    return ref_goal_Point;
}

pair<double, double> get_GoalHeadingPoint_double(const AstarOsqp::State& g_state){
    return make_pair(g_state.x - cos(g_state.heading)*5, g_state.y - sin(g_state.heading)*5);
}


void setBox(vector<double>& box, double bound){
    box.clear();
    for(int i = 0; i < 4; i++){
        box.push_back(bound);
    }
}

int main(int argc, char** argv)
{
    // init ros node
    ros::init(argc, argv, "path_opyimization");
    ros::NodeHandle n("~");

    //load image
    string image_dir = ros::package::getPath("astar_osqp");
    string image_file = "gridmap.png";
    image_dir.append("/" + image_file);

    //init grid_map
    cv::Mat img_src = cv::imread(image_dir, CV_8UC1);
    double resolution = 1;
    unsigned char OCCUPY = 0;
    unsigned char FREE = 255;
    grid_map::GridMap grid_map(std::vector<std::string>{"obstacle", "distance"});
    grid_map::Position p(0.5*img_src.rows - 0.5, 0.5*img_src.cols - 0.5);
    grid_map::GridMapCvConverter::initializeFromImage(img_src, resolution, grid_map, p);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(img_src, "obstacle", grid_map, OCCUPY, FREE, 1);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> binary = grid_map.get("obstacle").cast<unsigned char>();
    cv::distanceTransform(eigen2cv(binary), eigen2cv(grid_map.get("distance")), CV_DIST_L2, CV_DIST_MASK_PRECISE);
    grid_map.get("distance") *= resolution;
    grid_map.setFrameId("/map");

    //pub and sub
    ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);
    ros::Subscriber ref_sub = n.subscribe("/clicked_point", 1, referenceCb);
    ros::Subscriber start_sub = n.subscribe("/initialpose", 1, startCb);
    ros::Subscriber end_sub = n.subscribe("/move_base_simple/goal", 1, goalCb);

    //vis_tools_param
    ros_viz_tools::RosVizTools markers(n, "markers");
    string marker_frame_id = "/map";
    
    //init aStar
    cv::Mat Mask;
    AstarOsqp::Astar A;
    A.InitAstar(img_src, Mask);   
    vector<cv::Point> obs = A.getObstacle();

    //kdtree
    pointVec obsPoints;
    point_t pt;
    for(size_t i = 0; i < obs.size(); ++i)
    {
        pt.push_back(obs[i].x);
        pt.push_back(obs[i].y);
        obsPoints.push_back(pt);
        pt.clear();
    }
    KDTree obsTree(obsPoints);

    //astar result
    vector<pair<double,double>> pathlist;
    vector<pair<double,double>> Totalpathlist;
    
    //interpolation
    double sum_dis = 0.0;
    double per_dis = 2.0; 
    vector<pair<double,double>> raw_x_traj;
    vector<pair<double,double>> raw_y_traj;
    Interpolation1D inter_x_traj;
    Interpolation1D inter_y_traj;
    vector<pair<double,double>> inter_traj;
    vector<pair<double,double>> tmp_traj;
    vector<pair<double,double>> plan_traj;

    //corridor
    AstarOsqp::corridor C;
    C.initCollsion(obsTree, img_src.cols, img_src.rows);

    //time
    clock_t s_time, e_time;

    //Osqp param
    FemPosDeviationSqpOsqpInterface solver;
    vector<pair<double, double>> raw_point2d;
    vector<vector<double>> bound_boxes;
    vector<double> box;
    vector<pair<double,double>> opt_path;

    //assitant vairable
    double dx,dy,dis;

    ros::Rate rate(30);
    
    //loop
    while(n.ok()){
        ros::Time time = ros::Time::now();
        //vis_tool_marker
        markers.clear();
        int id = 0;

        //double click to clear ref_point
        if(ref_point_plot.size() >= 2){
            const auto &p1 = ref_point_plot[ref_point_plot.size() - 2];
            const auto &p2 = ref_point_plot.back();
            if (distance(p1, p2) <= 0.001) {
                ref_point_plot.clear();
                reference_rcv = false;
            }

        }

        //vis ref_point
        visualization_msgs::Marker ref_marker = markers.newSphereList(1, "reference point", id++, ros_viz_tools::RED, marker_frame_id);
        for(size_t i = 0; i != ref_point_plot.size(); i++){
            geometry_msgs::Point p;
            p.x = ref_point_plot[i].x;
            p.y = ref_point_plot[i].y;
            p.z = 1.0;
            ref_marker.points.push_back(p);
        }
        markers.append(ref_marker);

        //vis start and goal (x,y,heading)
        geometry_msgs::Vector3 scale;
        scale.x = 2.0;
        scale.y = 0.3;
        scale.z = 0.3;
        geometry_msgs::Pose start_pose;
        start_pose.position.x = start_state.x;
        start_pose.position.y = start_state.y;
        start_pose.position.z = 1.0;
        auto start_quat = tf::createQuaternionFromYaw(start_state.heading);
        start_pose.orientation.x = start_quat.x();
        start_pose.orientation.y = start_quat.y();
        start_pose.orientation.z = start_quat.z();
        start_pose.orientation.w = start_quat.w();
        visualization_msgs::Marker start_marker =markers.newArrow(scale, start_pose, "start point", id++, ros_viz_tools::CYAN, marker_frame_id);
        markers.append(start_marker);
        geometry_msgs::Pose end_pose;
        end_pose.position.x = end_state.x;
        end_pose.position.y = end_state.y;
        end_pose.position.z = 1.0;
        auto end_quat = tf::createQuaternionFromYaw(end_state.heading);
        end_pose.orientation.x = end_quat.x();
        end_pose.orientation.y = end_quat.y();
        end_pose.orientation.z = end_quat.z();
        end_pose.orientation.w = end_quat.w();
        visualization_msgs::Marker end_marker =markers.newArrow(scale, end_pose, "end point", id++, ros_viz_tools::CYAN, marker_frame_id);
        markers.append(end_marker);

        //start_ref_point and end_ref_point for heading
        cv::Point start_ref_point = get_StartHeadingPoint(start_state);
        pair<double, double> start_ref_point_double = get_StartHeadingPoint_double(start_state);

        cv::Point end_ref_point = get_GoalHeadingPoint(end_state);
        pair<double, double> end_ref_point_double = get_GoalHeadingPoint_double(end_state);

        vector<cv::Point> ref_points;
        ref_points.push_back(start_ref_point);
        for(size_t i = 0; i < ref_point_plot.size(); ++i){
            cv::Point ref_point;
            ref_point.x = ref_point_plot[i].x;
            ref_point.y = ref_point_plot[i].y;
            ref_points.push_back(ref_point);
        }
        ref_points.push_back(end_ref_point);

        //planning
        if(start_state_rcv && end_state_rcv && reference_rcv){

            //init refPoint kdtree
        /*    pointVec refPoints;
            point_t ref_pt;
            for(size_t i = 0; i < ref_point_plot.size(); ++i)
            {
                ref_pt.push_back(ref_point_plot[i].x);
                ref_pt.push_back(ref_point_plot[i].y);
                refPoints.push_back(ref_pt);
                ref_pt.clear();
            }
            KDTree refTree(refPoints); */

            //get ref path
            for(int i = 0; i < 5; i++){
                pathlist.push_back(make_pair(start_state.x + cos(start_state.heading)*i, start_state.y + sin(start_state.heading)*i));
            }
            dx = (start_ref_point.x - start_ref_point_double.first) / 5;
            dy = (start_ref_point.y - start_ref_point_double.second) / 5;
            for(int i = 0; i < 5; i++){
                pathlist.push_back(make_pair(start_ref_point_double.first + dx * i, start_ref_point_double.second + dy * i));
            } 
            Totalpathlist.insert(Totalpathlist.end(), pathlist.begin(), pathlist.end());
            pathlist.clear();

            for(size_t i = 0; i < ref_points.size()-1; ++i){
                pathlist = A.pathPlanning(ref_points[i], ref_points[i+1]);
                Totalpathlist.insert(Totalpathlist.end(), pathlist.begin(), pathlist.end());
            }
            pathlist.clear();

            dx = (end_ref_point_double.first - end_ref_point.x) / 5;
            dy = (end_ref_point_double.second - end_ref_point.y) / 5;
            for(int i = 0; i < 5; i++){
                pathlist.push_back(make_pair(end_ref_point.x + dx * i, end_ref_point.y + dy * i));
            }
            for(int i = 4; i >= 0; i--){
                pathlist.push_back(make_pair(end_state.x - cos(end_state.heading)*i, end_state.y - sin(end_state.heading)*i));
            }
            pathlist.push_back(make_pair(end_state.x, end_state.y));
            Totalpathlist.insert(Totalpathlist.end(), pathlist.begin(), pathlist.end());
 
            //get inter_path
            for(size_t i = 0; i < Totalpathlist.size()/3; i++){
                tmp_traj.push_back(Totalpathlist[i*3]);
            }
            tmp_traj.push_back(Totalpathlist[Totalpathlist.size()-1]);

            raw_x_traj.push_back(make_pair(sum_dis, tmp_traj.front().first));
            raw_y_traj.push_back(make_pair(sum_dis, tmp_traj.front().second));

            for(size_t i = 1; i < tmp_traj.size();++i){
                sum_dis += AstarOsqp::distance(tmp_traj[i],tmp_traj[i-1]);
                raw_x_traj.push_back(make_pair(sum_dis, tmp_traj[i].first));
                raw_y_traj.push_back(make_pair(sum_dis, tmp_traj[i].second));
            }

            inter_x_traj.Init(raw_x_traj);
            inter_y_traj.Init(raw_y_traj);            

            for(float count_dis = 0; count_dis < sum_dis; count_dis+=per_dis)
            {
                double tmp_x = inter_x_traj.Interpolate(count_dis);
                double tmp_y = inter_y_traj.Interpolate(count_dis);
                inter_traj.push_back(make_pair(tmp_x, tmp_y));
            }
            inter_traj.push_back(make_pair(end_state.x, end_state.y));

            //more points
            for(size_t i = 0; i < inter_traj.size()-1; ++i){
                dx = (inter_traj[i+1].first - inter_traj[i].first)/2;
                dy = (inter_traj[i+1].second - inter_traj[i].second)/2;
                for(int k = 0; k < 2; k++)
                {
                    plan_traj.push_back(make_pair(inter_traj[i].first + k * dx, inter_traj[i].second + k * dy));
                }
            }
            plan_traj.push_back(make_pair(inter_traj[inter_traj.size()-1].first, inter_traj[inter_traj.size()-1].second));

          /*  for(size_t i = 0; i < plan_traj.size(); i++){
                cout << "x:" << plan_traj[i].first << " y:" << plan_traj[i].second << endl;
            } */

            //get corridor
            s_time = clock();
            C.update(plan_traj);
            e_time = clock();
            cout << "update time is: " <<(double)(e_time - s_time) / CLOCKS_PER_SEC << "s" << endl;

            //osqp
            point_t tmp;
            for(int i =0; i<plan_traj.size(); ++i){
                raw_point2d.push_back(make_pair(plan_traj[i].first, plan_traj[i].second));
            }
            solver.set_ref_points(raw_point2d);

            for(size_t j = 0 ; j < 10; ++j)
            {   
                setBox(box, 0.3);
                bound_boxes.push_back(box);
            }
            for(size_t j = 10  ; j < plan_traj.size()-10; ++j)
            {   
               /* tmp.clear();
                tmp.push_back(plan_traj[j].first);
                tmp.push_back(plan_traj[j].second);
                auto res = refTree.nearest_point(tmp);
                dis = sqrt(pow(tmp[0] - res[0], 2) + pow(tmp[1] - res[1], 2)); */
                bound_boxes.push_back(C.boxes[j]);
            }
            for(size_t j = plan_traj.size()- 10; j < plan_traj.size(); ++j)
            {
                setBox(box, 0.3);
                bound_boxes.push_back(box);
            }
            
            solver.set_boundsBox(bound_boxes);

            if(!solver.Solve()){
                cout << "failure" << endl;
            }

            opt_path = solver.opt_xy();
            start_state_rcv = false;
        }

        bool opt_ok = false;

        //test
      /*  if(test_flag){
            C.update(test_traj);
            test_flag = false;
        } */

        //define path_color
        ros_viz_tools::ColorRGBA path_color;
        path_color.r = 1.0;
        path_color.g = 0.0;
        path_color.b = 0.0;

        //vis astar path
        visualization_msgs::Marker result_path_marker = markers.newLineStrip(0.2, "ref path", id++, path_color, marker_frame_id);
        for(size_t i = 0; i < Totalpathlist.size() ; ++i){
            geometry_msgs::Point p;
            p.x = Totalpathlist[i].first;
            p.y = Totalpathlist[i].second;
            p.z = 1.0;
            result_path_marker.points.push_back(p);
            path_color.a = 1;
            result_path_marker.colors.emplace_back(path_color);
        }
        markers.append(result_path_marker);

        //vis opt path
        path_color.r = 0.063;
        path_color.g = 0.305;
        path_color.b = 0.545;
        visualization_msgs::Marker opt_path_marker = markers.newLineStrip(0.5, "opt path", id++, path_color, marker_frame_id);
        for(size_t i = 0; i < opt_path.size(); ++i){
            geometry_msgs::Point p;
            p.x = opt_path[i].first;
            p.y = opt_path[i].second;
            p.z = 1.0;
            opt_path_marker.points.push_back(p);
            path_color.a = 1;
            opt_path_marker.colors.emplace_back(path_color);
        }
        markers.append(opt_path_marker); 

        //vis inter path
        path_color.r = 0.0;
        path_color.g = 0.9;
        path_color.b = 0.1;
        visualization_msgs::Marker inter_path_marker = markers.newLineStrip(0.2, "inter path", id++, path_color, marker_frame_id);
        for(size_t i = 0; i < inter_traj.size() ; ++i){
            geometry_msgs::Point p;
            p.x = inter_traj[i].first;
            p.y = inter_traj[i].second;
            p.z = 1.0;
            inter_path_marker.points.push_back(p);
            path_color.a = 1;
            inter_path_marker.colors.emplace_back(path_color);
        }
        markers.append(inter_path_marker); 

         //vis corridor
        path_color.r = 0.1;
        path_color.g = 0.1;
        path_color.b = 0.8;
        for(size_t i = 0; i < C.vis_boxes.size(); i++){
            visualization_msgs::Marker corridor_marker = markers.newLineStrip(1, "corridor path", id++, path_color, marker_frame_id);
            for(size_t j = 0; j <= C.vis_boxes[i].size() ; ++j){
                geometry_msgs::Point p;
                if(j == 0){
                    p.x = C.vis_boxes[i][0];
                    p.y = C.vis_boxes[i][1];
                    p.z = 1.0;
                }
                if(j == 1){
                    p.x = C.vis_boxes[i][0];
                    p.y = C.vis_boxes[i][3];
                    p.z = 1.0;
                }
                if(j == 2){
                    p.x = C.vis_boxes[i][2];
                    p.y = C.vis_boxes[i][3];
                    p.z = 1.0;
                }
                if(j == 3){
                    p.x = C.vis_boxes[i][2];
                    p.y = C.vis_boxes[i][1];
                    p.z = 1.0;
                }
                if(j == 4){
                    p.x = C.vis_boxes[i][0];
                    p.y = C.vis_boxes[i][1];
                    p.z = 1.0;
                }
                corridor_marker.points.push_back(p);
                path_color.a = 1;
                corridor_marker.colors.emplace_back(path_color);
            }
            markers.append(corridor_marker); 
        }



        grid_map.setTimestamp(time.toNSec());
        nav_msgs::OccupancyGrid message;
        grid_map::GridMapRosConverter::toOccupancyGrid(grid_map, "obstacle", FREE, OCCUPY, message);
        map_publisher.publish(message);
        markers.publish();

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}