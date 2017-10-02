#include "Trajectory.h"

#include "Eigen-3.3/Eigen/Dense"
#include "Constants.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

Trajectory::Trajectory() {}

Trajectory::Trajectory(vector<double> start_s, vector<double> end_s, double s_T, vector<double> start_d,
                       vector<double> end_d, double d_T, SensorFusion *sensor_fusion, Map *map){

//    vector<double> start_s = {};
//    vec
    this->s_coeff = JMT(start_s, end_s, s_T);
    this->d_coeff = JMT(start_d, end_d, d_T);
    this->s_duration = s_T;
    this->d_duration = d_T;

    this->end_s_ = end_s;
    this->end_d_ = end_d;

    this->sensorFusion = sensor_fusion;
    this->map = map;

}

vector<double> Trajectory::JMT(const vector<double> &start, const vector<double> &end, const double T) {
/*
Calculate the Jerk Minimizing Trajectory that connects the initial state
to the final state in time T.

INPUTS

start - the vehicles start location given as a length three array
corresponding to initial values of [s, s_dot, s_double_dot]

end - the desired end state for vehicle. Like "start" this is a
length three array.

T - The duration, in seconds, over which this maneuver should occur.

OUTPUT
an array of length 6, each value corresponding to a coefficent in the polynomial
s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

EXAMPLE

> JMT( [0, 10, 0], [10, 10, 0], 1)
[0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
*/

    MatrixXd A = MatrixXd(3, 3);

    double T_2 = T*T;
    double T_3 = T_2*T;
    double T_4 = T_3*T;
    double T_5 = T_4*T;

    A << T_3,   T_4,    T_5,
         3*T_2, 4*T_3,  5*T_4,
         6*T,   12*T_2, 20*T_3;

    MatrixXd B = MatrixXd(3,1);
    B << end[0]-(start[0]+start[1]*T+.5*start[2]*T_2),
         end[1]-(start[1]+start[2]*T),
         end[2]-start[2];


    MatrixXd Ai = A.inverse();

    MatrixXd C = Ai*B;

    vector<double> result = {start[0], start[1], .5*start[2]};
    for(int i = 0; i < C.size(); i++) {
        result.push_back(C.data()[i]);
    }

    return result;
}

double Trajectory::position_at(const vector<double> &a, const double t) {
    const auto t2 = t * t;
    const auto t3 = t2 * t;
    const auto t4 = t3 * t;
    const auto t5 = t4 * t;
    return a[0] + a[1] * t + a[2] * t2 + a[3] * t3 + a[4] * t4 + a[5] * t5;
}

double Trajectory::velocity_at(const vector<double> &a, const double t) {
    const auto t2 = t * t;
    const auto t3 = t2 * t;
    const auto t4 = t3 * t;
    return a[1] + 2. * a[2] * t + 3. * a[3] * t2 + 4. * a[4] * t3 + 5. * a[5] * t4;
}

double Trajectory::acceleration_at(const vector<double> &a, const double t) {
    const auto t2 = t * t;
    const auto t3 = t2 * t;
    return 2 * a[2] + 6 * a[3] * t + 12 * a[4] * t2 + 20 * a[5] * t3;
}

double Trajectory::jerk_at(const vector<double> &a, const double t) {
    return 6 * a[3] + 24 * a[4] * t + 60 * a[5] * t * t;
}

/* Calculating total jerk and check for max jerk */


double Trajectory::acceleration_cost() {
    return 0;
}

double Trajectory::velocity_cost() {
    double total = 0;
    int cnt = 0;
    double t = 0.0;
    while(t < COST_HORIZON_T){
        double v_s = velocity_at(s_coeff, min(t, s_duration));
        double v_d = velocity_at(d_coeff, min(t, d_duration));
        double v = sqrt(v_s*v_s+v_d*v_d);
        total += pow(v-MAX_SPEED, 2);
        t+=COST_DT;
        cnt++;
    }
    return total/cnt;
}

double Trajectory::max_velocity_cost() {
    double t = 0.0;

    double sd_xy_coeff = 1;

    for(double t = COST_DT; t<min(s_duration, d_duration); t+=min(s_duration, d_duration)/3){
        double s0 = position_at(s_coeff, min(t - COST_DT, s_duration));
        double s1 = position_at(s_coeff, min(t, s_duration));
        double d0 = position_at(d_coeff, min(t - COST_DT, d_duration));
        double d1 = position_at(d_coeff, min(t, d_duration));

        double dist_sd = sqrt(pow(s0-s1, 2) + pow(d0-d1, 2));

        auto xy0 = map->getXY(s0, d0);
        auto xy1 = map->getXY(s1, d1);

        double dist_xy = sqrt(pow(xy0[0]-xy1[0], 2) + pow(xy0[1]-xy1[1], 2));

        sd_xy_coeff = max(sd_xy_coeff, dist_xy/dist_sd);
    }


    while(t < COST_HORIZON_T){

        double v_s = velocity_at(s_coeff, min(t, s_duration));
        double v_d = velocity_at(d_coeff, min(t, d_duration));
        double v = sqrt(v_s*v_s+v_d*v_d);


        if(v*sd_xy_coeff > MAX_SPEED)
            return 1;
        t+=COST_DT;
    }
    return 0;

//    double t = COST_DT;
//    while(t < COST_HORIZON_T){
//        double s0 = position_at(s_coeff, min(t - COST_DT, s_duration));
//        double s1 = position_at(s_coeff, min(t, s_duration));
//        double d0 = position_at(d_coeff, min(t - COST_DT, d_duration));
//        double d1 = position_at(d_coeff, min(t, d_duration));
//
//        auto xy0 = map->getXY(s0, d0);
//        auto xy1 = map->getXY(s1, d1);
//
//        double dist = sqrt(pow(xy0[0]-xy1[0], 2) + pow(xy0[1]-xy1[1], 2));
//        double v = dist/COST_DT;
//
//        if(v > MAX_SPEED)
//            return 1;
//        t+=COST_DT;
//    }
//    return 0;
}


double Trajectory::lane_potential_cost() {
    double lane_speed = 100.0;
    double lane_free_distance = 100.0;

    int lane = map->dToLane(end_d_[0]);
    double s = position_at(s_coeff, 0);

    for(auto vehicle: sensorFusion->vehicles){
        if(lane == vehicle.lane && s < vehicle.s){
            if(lane_free_distance > vehicle.s-s) {
                lane_free_distance = min(lane_free_distance, vehicle.s - s);
                lane_speed = min(lane_speed, vehicle.v);
            }
        }
    }

    return 1/lane_speed  + 1/lane_free_distance;
}

double Trajectory::collision_cost() {
    double t = 0;

    while(t < max(s_duration, d_duration)){

        double s = position_at(s_coeff, t);
        double d = position_at(d_coeff, t);

        for(auto vehicle: sensorFusion->vehicles){
            if(vehicle.collide(s, d, t)){
                return 1;
            }
        }
        t+= COST_DT;
    }
    return 0;
}

double logistic(double x){
    // A function that returns a value between 0 and 1 for x in the range[0, infinity] and - 1 to 1 for x in
    // the range[-infinity, infinity]. Useful for cost functions.
    return 2.0 / (1 + exp(-x)) - 1.0;
}

double Trajectory::safety_cost() {
    double t = 0;
    double safety_cost = 0;
    while(t < max(s_duration, d_duration)) {
        double s = position_at(s_coeff, t);
        double d = position_at(d_coeff, t);
        for (auto vehicle: sensorFusion->vehicles) {
            int ind = t/COST_DT;
            if(ind < vehicle.predicted_s.size()) {
                double d_s = abs(s - vehicle.predicted_s[t / COST_DT]);
                double d_d = abs(d - vehicle.d);

                if(d_s < 8 && d_d < 4) {
                    double dist = sqrt(d_s * d_s + d_d * d_d);
                    safety_cost += logistic(2 * CAR_WIDTH / dist);
                }
            }
        }
        t+= COST_DT;
    }
    safety_cost = safety_cost / t;
    return safety_cost;
}

double Trajectory::get_s_at_t(const double t) {
    return position_at(s_coeff, min(t, s_duration));
}

double Trajectory::get_d_at_t(const double t) {
    return position_at(d_coeff, min(t, d_duration));
}

double Trajectory::get_vs_at_t(const double t) {
    return velocity_at(s_coeff, min(t, s_duration));
}

double Trajectory::get_vd_at_t(const double t) {
    return velocity_at(d_coeff, min(t, d_duration));
}

double Trajectory::get_as_at_t(const double t) {
    return acceleration_at(s_coeff, min(t, s_duration));
}

double Trajectory::get_ad_at_t(const double t) {
    return acceleration_at(d_coeff, min(t, d_duration));
}

double Trajectory::jerk_s_cost() {
    double total = 0;
    int cnt = 0;
    double t = 0.0;
    while(t < COST_HORIZON_T){
        double jerk = jerk_at(s_coeff, min(t, s_duration));
        total += jerk*jerk;
        t+=COST_DT;
        cnt++;
    }
    return total/cnt;
}

double Trajectory::jerk_d_cost() {
    double total = 0;
    int cnt = 0;
    double t = 0.0;
    while(t < COST_HORIZON_T){
        double jerk = jerk_at(d_coeff, min(t, d_duration));
        total += jerk*jerk;
        t+=COST_DT;
        cnt++;
    }
    return total/cnt;
}

double Trajectory::max_jerk_s_cost() {
    double t = 0.0;
    while(t < COST_HORIZON_T){
        double jerk = jerk_at(s_coeff, min(t, s_duration));
        if(abs(jerk) > S_MAX_JERK)
            return 1;
        t+=COST_DT;
    }
    return 0;
}

double Trajectory::max_jerk_d_cost() {
    double t = 0.0;
    while(t < COST_HORIZON_T){
        double jerk = jerk_at(d_coeff, min(t, d_duration));
        if(abs(jerk) > D_MAX_JERK)
            return 1;
        t+=COST_DT;
    }
    return 0;
}

double Trajectory::cost() {
    if(total_cost < 0){
        /* As simple optimization - if collisiotion happenes, we don't need to evaluate other staff - trajectory is bad anyway */
        total_cost = COLLISION_WEIGHT*collision_cost();
        if(total_cost < 1.0 ) {
            total_cost += S_JERK_WEIGHT*jerk_s_cost();
            total_cost += D_JERK_WEIGHT*jerk_d_cost();
            total_cost += S_MAX_JERK_WEIGHT*max_jerk_s_cost();
            total_cost += D_MAX_JERK_WEIGHT*max_jerk_d_cost();

            total_cost += VELOCITY_WEIGHT*velocity_cost();
            total_cost += VELOCITY_MAX_WEIGHT*max_velocity_cost();
            total_cost += SAFETY_WEIGHT*safety_cost();
            total_cost += 1000*lane_potential_cost();

        }
        //  jerk_cost() + acceleration_cost() + velocity_cost() + safety_cost()  + lane_potential_cost(); //+ collision_cost()
    }
    return total_cost;
}

int Trajectory::lane() {
    return map->dToLane(end_d_[0]);
}

void Trajectory::print() {
    cout << "LANE: " << lane() << ", S: " << end_s_[0] << ", ST: "  <<  s_duration << ", D: " << end_d_[0] << ", DT: " << d_duration;
    cout << " COST: " << cost() << " | " << COLLISION_WEIGHT*collision_cost() << " " << S_JERK_WEIGHT*jerk_s_cost() << " " << D_JERK_WEIGHT*jerk_d_cost();
    cout << " " << S_MAX_JERK_WEIGHT*max_jerk_s_cost() << " " << D_MAX_JERK_WEIGHT*max_jerk_d_cost() << " " << VELOCITY_WEIGHT*velocity_cost();
    cout << " " << VELOCITY_MAX_WEIGHT*max_velocity_cost() << " " << 1000*lane_potential_cost() << endl;
}





