/*
 * @Author: cagik
 * @Date: 2021-12-05 11:24:02
 * @LastEditTime: 2021-12-14 17:00:42
 * @LastEditors: cagik
 * @Description: 
 * @FilePath: /astarOsqp_ws/src/astar_osqp/include/data_struct.h
 * cavata_iwnl
 */
#ifndef DATA_STRUCT_H
#define DATA_STRUCT_H

namespace  AstarOsqp{

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