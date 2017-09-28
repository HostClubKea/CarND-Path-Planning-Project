//
// Created by dmitr on 24.09.2017.
//

#include "Vehicle.h"

Vehicle::Vehicle(Map& map):map(map){
    this->lane = 1;
}

Trajectory Vehicle::update(const Telemetry &telemetry) {

    Trajectory trajectory = pathPlanner.plan(telemetry, map);

    return trajectory;
};
