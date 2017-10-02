//
// Created by dmitr on 28.09.2017.
//

#ifndef PATH_PLANNING_SENSORFUSION_H
#define PATH_PLANNING_SENSORFUSION_H


#include "OtherVehicle.h"

class SensorFusion {

public:
    vector<OtherVehicle> vehicles;

    void update(const Telemetry &telemetry, Map& map, int lap, double start_time);

    OtherVehicle leading_vehicle(int lane, double s);

    vector<OtherVehicle> leading_vehicles(int lane, double s);
};


#endif //PATH_PLANNING_SENSORFUSION_H


