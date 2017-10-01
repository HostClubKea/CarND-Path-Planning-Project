//
// Created by dmitr on 27.09.2017.
//

#ifndef PATH_PLANNING_OTHERVEHICLE_H
#define PATH_PLANNING_OTHERVEHICLE_H


#include "Telemetry.h"
#include "Map.h"


class OtherVehicle {

public:

    double s;
    double d;
    double v;
    int lane;

    vector<double> predicted_s;
    vector<double> trajectory;

    OtherVehicle(Map& map, SensorFusionData& sensor_fusion_data, double start_time);

    bool collide(double s, double d, double dt);
};


#endif //PATH_PLANNING_OTHERVEHICLE_H
