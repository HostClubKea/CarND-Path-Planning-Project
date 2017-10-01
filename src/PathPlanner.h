//
// Created by dmitr on 24.09.2017.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H


#include "Trajectory.h"
#include "Telemetry.h"
#include "Map.h"
#include "SensorFusion.h"


enum VehicleState { CHANGE_LANE, KEEP_LANE };

class PathPlanner {
private:
    double ref_vel = 0;

    VehicleState current_state = KEEP_LANE;
    int target_lane = 1;

    vector<double> saved_start_s;
    vector<double> saved_start_d;

    bool changeLane(int new_lane, const vector<SensorFusionData>& sensor_data, int prev_size, double car_s);
public:
    Trajectory plan(vector<double> start_s, vector<double> start_d, Map& map, SensorFusion& sensorFusion);
};


#endif //PATH_PLANNING_PATHPLANNER_H
