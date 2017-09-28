//
// Created by dmitr on 24.09.2017.
//

#include "PathPlanner.h"
#include "utils.hpp"
#include "OtherVehicle.h"

Trajectory PathPlanner::plan(const Telemetry &telemetry, Map &map) {

    if(current_state == CHANGE_LANE){
        current_state == KEEP_LANE;
    }

    vector<OtherVehicle> other_vehicles;
    for(auto sensorFusionData: telemetry.sensor_fusion_data){
        other_vehicles.push_back(OtherVehicle(map, sensorFusionData));
    }

    vector<Trajectory> trajectories;

    vector<double> start_s;
    vector<double> start_d;
    if(saved_start_s.size() == 0) {
        start_s = {telemetry.s, telemetry.v, 0}; //change for a
        start_d = {telemetry.d, 0, 0}; //anyway to track speed + acceleration
    } else {
        start_s = saved_start_s;
        start_d = saved_start_d;
    }

    cout << "start_s " << start_s[0] << "; " << start_s[1] << "; " << start_s[2] << endl;

    for (int i = 0; i < LANES; i++) {
        double target_d = map.laneToD(i);
        for (double d_t = MIN_D_T; d_t < MAX_D_T; d_t += STEP_D_T) {
            vector<double> end_d = {target_d, 0, 0};
            for (double v = MIN_SPEED; v < MAX_SPEED; v += STEP_SPEED) {
                for (double s_t = MIN_S_T; s_t < MAX_S_T; s_t += STEP_S_T) {
                    double base_target_s = start_s[0] + (start_s[1]+v)/2*s_t;
                   // cout << "base_target_s=" <<base_target_s << " for v=" << v << endl;
                    for(int j = -2; j < 3; j++){
                        vector<double> end_s = {base_target_s + base_target_s/10.0*j,v,0};
                        if(end_s[0] > start_s[0])
                            trajectories.push_back(Trajectory(start_s, end_s, s_t, start_d, end_d, d_t));
                    }
                }
            }
        }
    }

    cout << trajectories.size() << endl;

    auto min_score_trajectory = trajectories[0];
    double min_score = min_score_trajectory.cost();

    int trajectory_checked = 0;
    for(auto trajectory: trajectories){
        trajectory_checked++;
        if(trajectory.cost() < min_score){
            min_score = trajectory.cost();
            min_score_trajectory = trajectory;

        }
    }
    cout<< "min_score=" << min_score << endl;
    cout << "end_s " << min_score_trajectory.end_s_[0] << "; " << min_score_trajectory.end_s_[1] << "; " << min_score_trajectory.end_s_[2] << endl;
    // Define the actual (x,y) points for the path planner
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // Start with all of the previous path points from last time
    for (int i = 0; i < telemetry.previous_path_x.size(); i++) {
        next_x_vals.push_back(telemetry.previous_path_x[i]);
        next_y_vals.push_back(telemetry.previous_path_y[i]);
    }

    // Fill up the rest of our path planner after filling it with the previous points, here we will always output 50 points
    int i = 0;
    for (i = 0; i <= 50-telemetry.previous_path_x.size();  i++) {
        if(i*0.02 > min_score_trajectory.s_duration)
            break;
        double s = min_score_trajectory.get_s_at_t(i*0.02);
        double d = min_score_trajectory.get_d_at_t(i*0.02);
        vector<double> xy_point = map.getXY(s, d);
        cout << "s=" << s <<"; d=" << d << "; x=" << xy_point[0] <<"; y=" << xy_point[1] << "; v=" << min_score_trajectory.get_vs_at_t(i*0.2)<< endl;

        next_x_vals.push_back(xy_point[0]);
        next_y_vals.push_back(xy_point[1]);
    }
        saved_start_s = {min_score_trajectory.get_s_at_t(i*0.02), min_score_trajectory.get_vs_at_t(i*0.02), min_score_trajectory.get_as_at_t(i*0.02)};
        saved_start_d = {min_score_trajectory.get_d_at_t(i*0.02), min_score_trajectory.get_vd_at_t(i*0.02), min_score_trajectory.get_ad_at_t(i*0.02)};


//
//        int prev_size = telemetry.previous_path_x.size();
//
//    // TODO: Implement finite state machine
//    // States - Keep line, Line change left and Line change right
//    // No need for PLCR and PLCL because no final state => no real need to slowdown and wait for the gap
//
//    //Create a path planner that performs optimized lane changing, this means that the car only changes into a lane that improves its forward progress.
//
//    //Incorporate a controller such as PID or MPC that follows the Path Planner's output path. Note that since the output path
//    // contains not only desired location information but also the car's desired speed as varying spaced points. One idea is to extract
//    // the desired speed from the path and then feed that into the controller. Another idea is if working with an MPC is to change the cost function
//    // so instead of evaluating cost relative to how close you are to a path, instead evaluate by how close the car is to one of the associating points of the path's output.
//
//    double car_s = telemetry.s;
//
//    if (prev_size > 0) {
//        car_s = telemetry.end_path_s;
//    }
//
//    int lane = map.dToLane(telemetry.d);
//
//    bool too_close = false;
//    // Find ref_v to use
//    for (int i = 0; i < telemetry.sensor_fusion_data.size(); i++) {
//
//
//        //car is in my lane
//        float d = telemetry.sensor_fusion_data[i].d;
//        if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
//            double vx = telemetry.sensor_fusion_data[i].vx;
//            double vy = telemetry.sensor_fusion_data[i].vy;
//            double check_speed = sqrt(vx * vx + vy * vy);
//            double check_car_s = telemetry.sensor_fusion_data[i].s;
//
//            check_car_s += ((double) prev_size * 0.02 * check_speed);
//
//            if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
//                too_close = true;
//                if (lane == 0) {
//                    bool check_right = changeLane(1, telemetry.sensor_fusion_data, prev_size, car_s);
//                    if (check_right) {
//                        lane = 1;
//                    }
//
//                }
//                if (lane == 1) {
//                    bool check_left = changeLane(0, telemetry.sensor_fusion_data, prev_size, car_s);
//                    if (check_left) {
//                        lane = 0;
//                    } else {
//                        bool check_right = changeLane(2, telemetry.sensor_fusion_data, prev_size, car_s);
//                        if (check_right) {
//                            lane = 2;
//                        }
//                    }
//                }
//                if (lane == 2) {
//                    bool check_left = changeLane(1, telemetry.sensor_fusion_data, prev_size, car_s);
//                    if (check_left) {
//                        lane = 1;
//                    }
//                }
//
//            }
//
//        }
//
//
//    }
//
//    // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
//    // Later we will interpolate these waypoints with a spline and fill it in with more ponts that control spline
//
//    vector<double> ptsx;
//    vector<double> ptsy;
//
//    // Reference x,y, yaw states
//    double ref_x = telemetry.x;
//    double ref_y = telemetry.y;
//    double ref_yaw = deg2rad(telemetry.yaw);
//
//    // If previous size is almost empty, use the car as starting reference
//    if (prev_size < 2) {
//        // Use two points that make the path tangent to the car
//        double prev_car_x = telemetry.x - cos(telemetry.yaw);
//        double prev_car_y = telemetry.y - sin(telemetry.yaw);
//
//        ptsx.push_back(prev_car_x);
//        ptsx.push_back(telemetry.x);
//
//        ptsy.push_back(prev_car_y);
//        ptsy.push_back(telemetry.y);
//    }
//        // Use the previous path's end point as starting reference
//    else {
//        // Redefine reference state as previous path end point
//        ref_x = telemetry.previous_path_x[prev_size - 1];
//        ref_y = telemetry.previous_path_y[prev_size - 1];
//
//        double ref_x_prev = telemetry.previous_path_x[prev_size - 2];
//        double ref_y_prev = telemetry.previous_path_y[prev_size - 2];
//        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
//
//        // Use two points that make the path tangent to the previous path's end point
//        ptsx.push_back(ref_x_prev);
//        ptsx.push_back(ref_x);
//
//        ptsy.push_back(ref_y_prev);
//        ptsy.push_back(ref_y);
//
//    }
//
//    // In Frenet add evenly 30m spaced points ahead of the starting reference
////                            vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
////                            vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
////                            vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
//
//    vector<double> next_wp0 = map.getXY(car_s + 30, (2 + 4 * lane));
//    vector<double> next_wp1 = map.getXY(car_s + 60, (2 + 4 * lane));
//    vector<double> next_wp2 = map.getXY(car_s + 90, (2 + 4 * lane));
////
//    ptsx.push_back(next_wp0[0]);
//    ptsx.push_back(next_wp1[0]);
//    ptsx.push_back(next_wp2[0]);
//
//    ptsy.push_back(next_wp0[1]);
//    ptsy.push_back(next_wp1[1]);
//    ptsy.push_back(next_wp2[1]);
//
//    // Transform coordinates to local car's coordinates
//    for (int i = 0; i < ptsx.size(); i++) {
//        double shift_x = ptsx[i] - ref_x;
//        double shift_y = ptsy[i] - ref_y;
//        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
//        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
//    }
//
//    // Create a spline
//    tk::spline s;
//
//    // Set(x,y) points to the spline
//    s.set_points(ptsx, ptsy);
//
//    // Define the actual (x,y) points for the path planner
//    vector<double> next_x_vals;
//    vector<double> next_y_vals;
//
//    // Start with all of the previous path points from last time
//    for (int i = 0; i < telemetry.previous_path_x.size(); i++) {
//        next_x_vals.push_back(telemetry.previous_path_x[i]);
//        next_y_vals.push_back(telemetry.previous_path_y[i]);
//    }
//
//    // Calculate how to break up spline points so that we travel at our desired reference velocity
//    double target_x = 30.0;
//    double target_y = s(target_x);
//    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
//
//    double x_add_on = 0;
//
//
//    cout << "path size = " << 50 - telemetry.previous_path_x.size() << endl;
//
//    // Fill up the rest of our path planner after filling it with the previous points, here we will always output 50 points
//    for (int i = 1; i <= 50 - telemetry.previous_path_x.size(); i++) {
//
//
//
//        // Decrease or increase the speed incrementally
//        if (too_close) {
//            ref_vel -= 0.224;
//        } else if (ref_vel < 49.5) {
//            ref_vel += 0.224;
//        }
//
//        double N = (target_dist / (0.02 * ref_vel / 2.24));
//        double x_point = x_add_on + (target_x) / N;
//        double y_point = s(x_point);
//
//        x_add_on = x_point;
//
//        double x_ref = x_point;
//        double y_ref = y_point;
//
//        // Rotate back to normal after rotating it earlier
//        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
//        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
//
//        x_point += ref_x;
//        y_point += ref_y;
//
//        next_x_vals.push_back(x_point);
//        next_y_vals.push_back(y_point);
//
//    }
//


    Trajectory trajectory = min_score_trajectory;
    trajectory.set_next_x_vals(next_x_vals);
    trajectory.set_next_y_vals(next_y_vals);
    return trajectory;
}

bool PathPlanner::changeLane(int new_lane, const vector<SensorFusionData> &sensor_data, int prev_size, double car_s) {
    for (int i = 0; i < sensor_data.size(); i++) {
        float d = sensor_data[i].d;
        if (d < (2 + 4 * new_lane + 2) && d > (2 + 4 * new_lane - 2)) {
            double vx = sensor_data[i].vx;
            double vy = sensor_data[i].vy;
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_data[i].s;

            // Nearly there just have to get this part right

            // how far forward is the car
            check_car_s += ((double) prev_size * 0.02 * check_speed);

            if ((check_car_s - car_s < 30) && ((check_car_s - car_s) > -30)) {
                return false;
            }

        }
    }
    return true;
}