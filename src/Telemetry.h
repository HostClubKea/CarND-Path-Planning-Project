#ifndef PATH_PLANNING_TELEMETRY_H
#define PATH_PLANNING_TELEMETRY_H

#include "json.hpp"
#include <vector>

using namespace std;
using json = nlohmann::json;

struct SensorFusionData {
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
};

class Telemetry {
public:
    /* Position in Cartesian coordinates */
    double x, y;

    /* Orientation in radians */
    double yaw;

    /* Position in Frenet coordinates */
    double s, d;

    /* Current linear speed */
    double v;

    // Previous path data given to the Planner
    vector<double> previous_path_x, previous_path_y;

    // Previous path's end s and d values
    double end_path_s, end_path_d;

    // Sensor Fusion Data
    vector<SensorFusionData> sensor_fusion_data;

    Telemetry(const nlohmann::json &telemetry);
};

#endif //PATH_PLANNING_TELEMETRY_H
