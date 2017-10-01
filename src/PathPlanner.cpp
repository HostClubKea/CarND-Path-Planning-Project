//
// Created by dmitr on 24.09.2017.
//

#include "PathPlanner.h"
#include "utils.hpp"
#include "OtherVehicle.h"
#include "Constants.h"


Trajectory PathPlanner::plan(vector<double> start_s, vector<double> start_d, Map &map, SensorFusion &sensorFusion) {

    if(current_state == CHANGE_LANE){
        current_state == KEEP_LANE;
    }

    vector<Trajectory> trajectories;

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
                            trajectories.push_back(Trajectory(start_s, end_s, s_t, start_d, end_d, d_t, &sensorFusion));
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

    return min_score_trajectory;

}
