//
// Created by dmitr on 24.09.2017.
//

#include "Vehicle.h"
#include "Constants.h"

Vehicle::Vehicle(Map& map):map(map){
    this->lane = 1;
    this->length = 0;
}

vector<vector<double>> Vehicle::update(const Telemetry &telemetry) {
    int old_size = 0;
    if(telemetry.previous_path_x.size() != 0)
        old_size = 50 - (length - telemetry.previous_path_x.size());




    if(start_s.size() == 0) {
        start_s = {telemetry.s, telemetry.v, 0}; //change for a
        start_d = {telemetry.d, 0, 0}; //anyway to track speed + acceleration
    }
    int lap = floor(start_s[0]/MAP_MAX_S);

    sensorFusion.update(telemetry, map, lap, old_size*CONTROL_DT);

    Trajectory trajectory = pathPlanner.plan(start_s, start_d, map, sensorFusion);

    cout<< "min_score=" << trajectory.cost() << endl;
    cout << "end_s " << trajectory.end_s_[0] << "; " << trajectory.end_s_[1] << "; " << trajectory.end_s_[2] << endl;
    // Define the actual (x,y) points for the path planner
    vector<double> next_x_vals;
    vector<double> next_y_vals;


    cout << "old_Size=" << old_size << endl;

    // Start with all of the previous path points from last time
    for (int i = 0; i < old_size/*telemetry.previous_path_x.size()*/; i++) {
        next_x_vals.push_back(telemetry.previous_path_x[i]);
        next_y_vals.push_back(telemetry.previous_path_y[i]);
    }

    cout << 1 << endl;
    cout <<  max(trajectory.s_duration, trajectory.d_duration) << endl;
    // Fill up the rest of our path planner after filling it with the previous points, here we will always output 50 points
    double t = CONTROL_DT;
    while( t < 5) //max(trajectory.s_duration, trajectory.d_duration))
    {
        double s = trajectory.get_s_at_t(t);
        double d = trajectory.get_d_at_t(t);
        vector<double> xy_point = map.getXY(s, d);
        next_x_vals.push_back(xy_point[0]);
        next_y_vals.push_back(xy_point[1]);
        t += CONTROL_DT;

    }

    cout << 2 << endl;
//    int i = 0;
//    for (i = 0; i <= 50-telemetry.previous_path_x.size();  i++) {
//        if(i*0.02 > max(trajectory.s_duration, trajectory.d_duration))
//            break;
//        double s = trajectory.get_s_at_t(i*CONTROL_DT);
//        double d = trajectory.get_d_at_t(i*CONTROL_DT);
//        vector<double> xy_point = map.getXY(s, d);
//        cout << "s=" << s <<"; d=" << d << "; x=" << xy_point[0] <<"; y=" << xy_point[1] << "; v=" << trajectory.get_vs_at_t(i*CONTROL_DT)<< endl;
//
//        next_x_vals.push_back(xy_point[0]);
//        next_y_vals.push_back(xy_point[1]);
//    }

    int i = 50 - old_size;

    start_s = {trajectory.get_s_at_t(i*CONTROL_DT), trajectory.get_vs_at_t(i*CONTROL_DT), trajectory.get_as_at_t(i*CONTROL_DT)};
    start_d = {trajectory.get_d_at_t(i*CONTROL_DT), trajectory.get_vd_at_t(i*CONTROL_DT), trajectory.get_ad_at_t(i*CONTROL_DT)};

    length = next_x_vals.size();

    return {next_x_vals, next_y_vals};
};
