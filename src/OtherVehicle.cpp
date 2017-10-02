//
// Created by dmitr on 27.09.2017.
//

#include "OtherVehicle.h"
#include "Constants.h"

/* Assumption - car don't change lane, and don't change velocity */
OtherVehicle::OtherVehicle(Map &map, SensorFusionData &sensor_fusion_data, int lap, double start_time) {
    this->lane = map.dToLane(sensor_fusion_data.d);
    this->d = map.laneToD(this->lane);
    this->v = sqrt(sensor_fusion_data.vx*sensor_fusion_data.vx + sensor_fusion_data.vy*sensor_fusion_data.vy);
    this->s = sensor_fusion_data.s  + lap*MAP_MAX_S + this->v*start_time;

    for(int i = 0; i < 50; i++){
        trajectory.push_back(this->s + i*COST_DT*this->v);
    }
}

bool OtherVehicle::collide(double s, double d, double dt) {
    double vehicle_s = trajectory[dt/COST_DT];
    if(abs(vehicle_s-s) <= CAR_LENGTH && abs(this->d - d) <= CAR_WIDTH)
        return true;
    return false;
}
