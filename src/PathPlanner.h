//
// Created by dmitr on 24.09.2017.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H


#include "Trajectory.h"
#include "Telemetry.h"
#include "Map.h"

enum VehicleState { CHANGE_LANE, KEEP_LANE };

class PathPlanner {
private:
    const int LANES = 3;
    const double MIN_SPEED = 20;
    const double MAX_SPEED = 49.5* 0.44704;;
    const double STEP_SPEED = 2.0;
    const double MAX_S_T = 10;
    const double MIN_S_T = 1;
    const double STEP_S_T = 1;
    const double MIN_D_T = 3;
    const double MAX_D_T = 5;
    const double STEP_D_T = 3;

    double ref_vel = 0;

    VehicleState current_state = KEEP_LANE;
    int target_lane = 1;

    vector<double> saved_start_s;
    vector<double> saved_start_d;

    bool changeLane(int new_lane, const vector<SensorFusionData>& sensor_data, int prev_size, double car_s);
public:
    Trajectory plan(const Telemetry& telemetry, Map& map);
};


#endif //PATH_PLANNING_PATHPLANNER_H
