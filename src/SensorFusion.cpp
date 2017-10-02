//
// Created by dmitr on 28.09.2017.
//

#include "SensorFusion.h"

void SensorFusion::update(const Telemetry &telemetry, Map& map, int lap, double start_time) {
    vehicles.clear();
    for(auto sensor_fusion_data: telemetry.sensor_fusion_data){
        vehicles.push_back(OtherVehicle(map, sensor_fusion_data, lap, start_time));
    }
}

vector<OtherVehicle> SensorFusion::leading_vehicles(int lane, double s) {
    return vector<OtherVehicle>();
}

//OtherVehicle SensorFusion::leading_vehicle(int lane, double s) {
////    for(auto vehicle: vehicles){
////        if(vehicle.lane == lane && )
////    }
////    return
//}
