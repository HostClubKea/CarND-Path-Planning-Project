#ifndef PATH_PLANNING_MPC_H
#define PATH_PLANNING_MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
public:
    MPC();

    virtual ~MPC();

    vector<double> solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, vector<double> weights);
};

#endif //PATH_PLANNING_MPC_H
