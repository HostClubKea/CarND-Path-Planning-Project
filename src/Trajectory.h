//
// Created by dmitr on 24.09.2017.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>

using namespace std;


class Trajectory {
private:
    /* Different weights constant for calculation cost*/
    constexpr static double S_JERK_WEIGHT = 1;
    constexpr static double S_MAX_JERK = 9.0;
    constexpr static double S_MAX_JERK_WEIGHT= 100000;

    constexpr static double D_JERK_WEIGHT = 1;
    constexpr static double D_MAX_JERK = 9.0;
    constexpr static double D_MAX_JERK_WEIGHT= 100000;

    constexpr static double COLLISION_WEIGHT= 1000000;

    constexpr static double VELOCITY_WEIGHT = 1;
    constexpr static double VELOCITY_MAX_WEIGHT = 100000;

    constexpr static double MAX_SPEED = 49.5*0.44704;

    constexpr static double COST_HORIZON_T = 10;
    constexpr static double COST_DT = 0.2;

    double total_cost = -1;

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    vector<double> s_coeff;
    vector<double> d_coeff;

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

    vector<double>& get_next_x_vals() { return next_x_vals; }
    vector<double>& get_next_y_vals() { return next_y_vals; }

    void set_next_x_vals(vector<double> next_x_vals) { this->next_x_vals = next_x_vals; }
    void set_next_y_vals(vector<double> next_y_vals) { this->next_y_vals = next_y_vals; }

    Trajectory(vector<double> start_s, vector<double> end_s, double s_T, vector<double> start_d, vector<double> end_d, double d_T);

    double cost();
};


#endif //PATH_PLANNING_TRAJECTORY_H