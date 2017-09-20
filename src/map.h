//
// Created by dmitr on 20.09.2017.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H


#include "spline.h"
#include <vector>
#include <math.h>

using namespace std;

//We are going on the same track over and over again, so there could be done some optimization
//map could be inited one time and reuse waypoints and spline could be reused if start_wp didn't changed

class Map {
protected:
    tk::spline s_x;
    tk::spline s_y;
    tk::spline s_dx;
    tk::spline s_dy;

    vector<double> maps_s;
    vector<double> maps_x;
    vector<double> maps_y;
    vector<double> maps_dx;
    vector<double> maps_dy;

    int start_wp = -1000;

public:
    void init(vector<double> maps_s, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx, vector<double> maps_dy);

    double distance(double x1, double y1, double x2, double y2);

    int closestWaypoint(double x, double y);

    int nextWaypoint(double x, double y, double theta);

    vector<double> getFrenet(double x, double y, double theta);

    vector<double> getXY(double s, double d);
};


#endif //PATH_PLANNING_MAP_H
