//
// Created by dmitr on 24.09.2017.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H


#include "Map.h"
#include "Telemetry.h"
#include "Trajectory.h"
#include "PathPlanner.h"

class Vehicle {
    /* Position in Cartesian coordinates */
    double x, y;

    /* Orientation in radians */
    double yaw;

    /* Position in Frenet coordinates */
    double s, d;

    /* Current linear speed */
    double v;

    /* Linear acceleretion */
    double a;

    /* Current lane. */
    int lane;

    Map& map;
    PathPlanner pathPlanner;

public:
    Vehicle(Map& map);

    Trajectory update(const Telemetry& telemetry);
};


#endif //PATH_PLANNING_VEHICLE_H
