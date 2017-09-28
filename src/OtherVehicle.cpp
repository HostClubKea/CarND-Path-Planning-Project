//
// Created by dmitr on 27.09.2017.
//

#include "OtherVehicle.h"

/* Assumption - car don't change lane, and don't change velocity */
OtherVehicle::OtherVehicle(const Map &map, SensorFusionData &sensor_fusion_data) {
    this->s = sensor_fusion_data.s;
    this->d = sensor_fusion_data.d;
    this->lane = map.dToLane(sensor_fusion_data.d);
    this->v = sqrt(sensor_fusion_data.vx*sensor_fusion_data.vx + sensor_fusion_data.vy*sensor_fusion_data.vy);


}
