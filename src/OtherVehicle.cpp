//
// Created by dmitr on 27.09.2017.
//

#include "OtherVehicle.h"
#include "Constants.h"

/* Assumption - car don't change lane, and don't change velocity */
OtherVehicle::OtherVehicle(Map &map, SensorFusionData &sensor_fusion_data, double start_time) {
    this->d = sensor_fusion_data.d;
    this->lane = map.dToLane(sensor_fusion_data.d);
    this->v = sqrt(sensor_fusion_data.vx*sensor_fusion_data.vx + sensor_fusion_data.vy*sensor_fusion_data.vy);
    this->s = sensor_fusion_data.s + this->v*start_time;

    for(int i = 0; i < 50; i++){
        trajectory.push_back(this->s + i*0.2*this->v);
    }
}

bool OtherVehicle::collide(double s, double d, double dt) {
    double vehicle_s = trajectory[dt/0.2];
    if(abs(vehicle_s-s) <= CAR_LENGTH && abs(this->d - d) <= CAR_WIDTH)
        return true;
    return false;
}
