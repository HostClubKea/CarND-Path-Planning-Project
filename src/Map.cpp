//
// Created by dmitr on 20.09.2017.
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <array>
#include "Map.h"
#include "Constants.h"

Map::Map(const string map_file) {
    // The max s value before wrapping around the track back to 0

    ifstream in_map_(map_file.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        maps_x.push_back(x);
        maps_y.push_back(y);
        maps_s.push_back(s);
        maps_dx.push_back(d_x);
        maps_dy.push_back(d_y);
    }

    vector<double> s_;
    vector<double> x_;
    vector<double> y_;
    vector<double> dx_;
    vector<double> dy_;

    s_.push_back(maps_s[maps_s.size()] - MAP_MAX_S);
    x_.push_back(maps_x[maps_s.size()]);
    y_.push_back(maps_y[maps_s.size()]);
    dx_.push_back(maps_dx[maps_s.size()]);
    dy_.push_back(maps_dy[maps_s.size()]);

    for(int i=0; i < maps_s.size(); i++){
        s_.push_back(maps_s[i]);
        x_.push_back(maps_x[i]);
        y_.push_back(maps_y[i]);
        dx_.push_back(maps_dx[i]);
        dy_.push_back(maps_dy[i]);
    }

    s_.push_back(MAP_MAX_S);
    x_.push_back(maps_x[0]);
    y_.push_back(maps_y[0]);
    dx_.push_back(maps_dx[0]);
    dy_.push_back(maps_dy[0]);

    s_x.set_points(s_,x_);
    s_y.set_points(s_,y_);
    s_dx.set_points(s_,dx_);
    s_dy.set_points(s_,dy_);

}


double Map::distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int Map::closestWaypoint(double x, double y) {

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < maps_x.size(); i++) {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;
}

int Map::nextWaypoint(double x, double y, double theta) {

    int closest_wp = closestWaypoint(x, y);

    double map_x = maps_x[closest_wp];
    double map_y = maps_y[closest_wp];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = abs(theta - heading);

    if (angle > M_PI / 4) {
        closest_wp++;
    }

    return closest_wp;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Map::getFrenet(double x, double y, double theta) {
    int next_wp = nextWaypoint(x, y, theta);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
        prev_wp = maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++) {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y using splines
vector<double> Map::getXY(double s, double d){
/*    int new_start_wp = -2;

    while(s > maps_s[new_start_wp+2] && (new_start_wp < (int)(maps_s.size()-1) ))
    {
        new_start_wp++;
    }

    // Update splines only if road segment is changed
    if(new_start_wp != start_wp){
        start_wp = new_start_wp;

        vector<double> s_;
        vector<double> x_;
        vector<double> y_;
        vector<double> dx_;
        vector<double> dy_;

        cout << "-------------------------" << endl;
        for(int i=0; i < 4; i++){
            int wp=(start_wp+i)%maps_x.size();
            s_.push_back(maps_s[wp]);
            x_.push_back(maps_x[wp]);
            y_.push_back(maps_y[wp]);
            dx_.push_back(maps_dx[wp]);
            dy_.push_back(maps_dy[wp]);
            cout << maps_s[wp] << " " << wp << " " << i << endl;
        }

        s_x.set_points(s_,x_);
        s_y.set_points(s_,y_);
        s_dx.set_points(s_,dx_);
        s_dy.set_points(s_,dy_);
    }*/

    while(s > MAP_MAX_S)
        s -= MAP_MAX_S;

    double x = s_x(s) + d*s_dx(s);
    double y = s_y(s) + d*s_dy(s);

    return {x,y};
}

double Map::laneToD(const int lane) {
    constexpr array<double, 3> laneCenters = { 2.0, 6.0, 10.0 }; //-9.75 -6 -2
    return laneCenters[lane];
}

int Map::dToLane(const double d) {
    if( d < 0 || d > 12) {
        return -1;
    }

    if(d <= 4)
        return 0;

    if(d <= 8)
        return 1;

    return 2;
}
