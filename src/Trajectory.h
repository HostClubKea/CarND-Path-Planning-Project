//
// Created by dmitr on 24.09.2017.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include "SensorFusion.h"

using namespace std;


class Trajectory {
private:
    /* Different weights constant for calculation cost*/
    double total_cost = -1;

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    vector<double> s_coeff;
    vector<double> d_coeff;

    SensorFusion* sensorFusion;
    Map* map;

    vector<double> JMT(const vector<double>& start, const vector<double>& end, const double t);

    double jerk_s_cost();
    double jerk_d_cost();
    double max_jerk_s_cost();
    double max_jerk_d_cost();

    double acceleration_cost();
    double velocity_cost();
    double max_velocity_cost();
    double collision_cost();
    /* Distance cost between cars*/
    double safety_cost();
    double lane_potential_cost();

    double position_at(const vector<double>& a, const double t);
    double velocity_at(const vector<double>& a, const double t);
    double acceleration_at(const vector<double>& a, const double t);
    double jerk_at(const vector<double>& a, const double t);

public:
    double s_duration;
    double d_duration;

    vector<double> end_s_;
    vector<double> end_d_;

    double get_s_at_t(const double t);
    double get_d_at_t(const double t);
    double get_vs_at_t(const double t);
    double get_vd_at_t(const double t);
    double get_as_at_t(const double t);
    double get_ad_at_t(const double t);

    Trajectory();

    Trajectory(vector<double> start_s, vector<double> end_s, double s_T, vector<double> start_d,
                   vector<double> end_d, double d_T, SensorFusion *sensor_fusion, Map *map);

    double cost();
    int lane();
    void print();
};


#endif //PATH_PLANNING_TRAJECTORY_H
