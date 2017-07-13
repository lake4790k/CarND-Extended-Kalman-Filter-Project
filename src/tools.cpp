#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    for (unsigned int i = 0; i < estimations.size(); ++i) {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
    MatrixXd Hj(3, 4);
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float xy2 = px * px + py * py;
    if (fabs(xy2) < 0.0001) {
        return Hj;
    }

    float sqrtXy2 = sqrt(xy2);
    float sqrtXy32 = xy2 * sqrtXy2;

    Hj << px / sqrtXy2, py / sqrtXy2, 0, 0,
            -py / xy2, px / xy2, 0, 0,
            py * (vx * py - vy * px) / sqrtXy32, px * (vy * px - vx * py) / sqrtXy32,
            px / sqrtXy2, py / sqrtXy2;

    return Hj;
}
