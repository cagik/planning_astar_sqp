/*
 * @Author: cagik
 * @Date: 2021-12-05 11:24:02
 * @LastEditTime: 2022-01-18 12:31:25
 * @LastEditors: cagik
 * @Description: 
 * @FilePath: /astarOsqp_ws/src/astar_osqp/include/data_struct.h
 * cavata_iwnl
 */
#ifndef DATA_STRUCT_H
#define DATA_STRUCT_H

namespace  AstarOsqp{

//描述车辆起始点以及终点状态，可以视为一个三维点（x,y,theta）
struct State
{
    State() = default;
    State(double x, double y, double head) : x(x), y(y), heading(head){}
    double x{};
    double y{};
    double heading{};
};

}

#endif