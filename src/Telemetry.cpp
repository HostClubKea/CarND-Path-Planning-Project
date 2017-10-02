#include "Telemetry.h"
#include "Constants.h"

Telemetry::Telemetry(const nlohmann::json &telemetry) {
    x = telemetry["x"];
    y = telemetry["y"];
    s = telemetry["s"];
    d = telemetry["d"];
    yaw = telemetry["yaw"];
    v = telemetry["speed"];
    v*=0.44704;

    auto prv_path_x = telemetry["previous_path_x"];
    auto prv_path_y = telemetry["previous_path_y"];

    for (int i = 0; i < prv_path_x.size(); i++) {
        previous_path_x.push_back(prv_path_x[i]);
        previous_path_y.push_back(prv_path_y[i]);
    }

    end_path_s = telemetry["end_path_s"];
    end_path_d = telemetry["end_path_d"];

    auto sensor_fusion = telemetry["sensor_fusion"];
    //sensor_fusion_data.resize(sensor_fusion.size());

    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        SensorFusionData sfd;
        sfd.id = sensor_fusion[i][0];
        sfd.x = sensor_fusion[i][1];
        sfd.y = sensor_fusion[i][2];
        sfd.vx = sensor_fusion[i][3];
     //   sfd.vx *= 0.44704;
        sfd.vy = sensor_fusion[i][4];
     //   sfd.vy *= 0.44704;
        sfd.s = sensor_fusion[i][5];
        sfd.d = sensor_fusion[i][6];
        sensor_fusion_data.push_back(sfd);
    }

};