#include "corridor.h"

namespace AstarOsqp{

void corridor::initCorridor(KDTree tree, double cols, double rows)
{
    obstacleTree_ = tree;
    cols_ = cols;
    rows_ = rows;
}

void corridor::test(){
    point_t t;
    t.clear();
    t.push_back(211.12);
    t.push_back(211.12);
    auto r = obstacleTree_.nearest_point(t);
    cout << "r_x :" << r[0] << "r_y" << r[1] << endl;

}

void corridor::update(const vector<pair<double, double>> traj){
    traj_ = traj;
    updateObsBox();
}

bool corridor::isBoxInBoundary(const vector<double>& box)
{
    return  box[0] > 0 &&
            box[1] > 0 &&
            box[2] < cols_ &&
            box[3] < rows_;
}

bool corridor::isPointInBox(const pair<double,double> point, const vector<double>& box)
{
    return  point.first > box[0] &&
            point.second > box[1] &&
            point.first < box[2] &&
            point.second < box[3];
}

bool corridor::isObstacleInBox(vector<double>& box, double margin){
    double x,y;
    double dis;
    double dx,dy;
    point_t pt;

    if(box[0] > box[2]){
        swap(box[0], box[2]);
        cout << "error" << endl;
    }

    if(box[1] > box[3]){
        swap(box[1], box[3]);
        cout << "error" << endl;
    }

    dx = (box[2] - box[0])/5;
    dy = (box[3] - box[1])/5;

    for(double i = box[0]; i < box[2] + 0.02; i = i + dx){
        for(double j = box[1]; j < box[3] + 0.02; j = j + dy ){
            x = i - 0.02;
            y = j - 0.02;

            pt.clear();
            pt.push_back(x);
            pt.push_back(y);

            auto res = obstacleTree_.nearest_point(pt);
            dis = sqrt(pow(pt[0] - res[0], 2) + pow(pt[1] - res[1], 2));
            if(dis < margin){
                return true;
            }
        }
    }
    return false;
}

void corridor::expandBox(vector<double>& box, double margin){
    vector<double> box_cand, box_update;
    vector<int> axis_cand{0, 1, 2, 3};

    int i = -1;
    int axis;
    while(!axis_cand.empty()){
        box_cand = box;
        box_update = box;

        while(!isObstacleInBox(box_update, margin) && isBoxInBoundary(box_update)){
            i++;
            if(i >= axis_cand.size()){
                i = 0;
            }
            axis = axis_cand[i];

            box = box_cand;
            box_update = box_cand;

            if(axis < 2){
                box_update[axis + 2] = box_cand[axis];
                box_cand[axis] = box_cand[axis] - 0.5;
                box_update[axis] = box_cand[axis];
            }
            else{
                box_update[axis - 2] = box_cand[axis];
                box_cand[axis] = box_cand[axis] + 0.5;
                box_update[axis] = box_cand[axis];
            }

       /*     if(box_update[0] > box_update[2]){
                swap(box_update[0], box_update[2]);
                cout << "error" << endl;
            }
            if(box_update[1] > box_update[3]){
                swap(box_update[0], box_update[2]);
                cout << "error" << endl;
            }
            if(box_cand[1] > box_cand[3]){
                swap(box_cand[0], box_cand[2]);
                cout << "error" << endl;
            }
            if(box_cand[1] > box_cand[3]){
                swap(box_cand[0], box_cand[2]);
                cout << "error" << endl;
            }  */

        
        }
        axis_cand.erase(axis_cand.begin() + i);
        if(i > 0){
            i --;
        }
        else{
            i = axis_cand.size()-1;
        }
    }
}


void corridor::updateObsBox()
{
    vector<double> box_prev{0,0,0,0};
    vector<double> box;
    double x, y, x_next, y_next;

    for(int i = 0; i < traj_.size()-1; i++){

        x = traj_[i].first;
        y = traj_[i].second; 

        x_next = traj_[i+1].first;
        y_next = traj_[i+1].second;

        box.clear();

        if(isPointInBox(make_pair(x_next,y_next), box_prev)){
            boxes.push_back(box_prev);
            continue;
        }

        //init box
        box.emplace_back(min(x,x_next));
        box.emplace_back(min(y,y_next));
        box.emplace_back(max(x,x_next));
        box.emplace_back(max(y,y_next));

     /*   if(box[2] == box[0] || box[4] == box[1]){
            box.clear();
            box.emplace_back(round(x - 0.5));
            box.emplace_back(round(y - 0.5));
            box.emplace_back(round(x + 0.5));
            box.emplace_back(round(y + 0.5));
            cout << "equal test" << endl;
        } */

     /*   if(isObstacleInBox(box,2)){
            box.clear();
            box.emplace_back(round(x - 0.2));
            box.emplace_back(round(y - 0.2));
            box.emplace_back(round(x + 0.2));
            box.emplace_back(round(y + 0.2));
            cout << "obs test" << endl;
        } */
        
        expandBox(box, 1.8);


        if(box[0] > box[2]){
            swap(box[0], box[2]);
            cout << "error" << endl;
        }

        if(box[1] > box[3]){
            swap(box[1], box[3]);
            cout << "error" << endl;
        } 



        vis_boxes.push_back(box);
        boxes.push_back(box);
        box_prev = box;
    }
    boxes.push_back(box_prev);
}

}