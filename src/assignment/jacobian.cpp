#include <iostream>
#include "../Eigen/Dense"

#include <vector>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd CalculateJacobian(const VectorXd &x_state);

int main() {

    /*
     * Compute the Jacobian Matrix
     */

    //predicted state  example
    //px = 1, py = 2, vx = 0.2, vy = 0.4
    VectorXd x_predicted(4);
    x_predicted << 1, 2, 0.2, 0.4;

    MatrixXd Hj = CalculateJacobian(x_predicted);

    cout << "Hj:" << endl << Hj << endl;

    return 0;
}

MatrixXd CalculateJacobian(const VectorXd &x_state) {

    MatrixXd Hj(3, 4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //TODO: YOUR CODE HERE

    if (px + py == 0) {
        cout << "division by 0";
        return Hj;
    }

    float xy2 = px * px + py * py;
    float sqrtXy2 = sqrt(xy2);
    float sqrtXy32 = pow(xy2, 1.5);

    Hj << px / sqrtXy2, py / sqrtXy2, 0, 0,
            -py / xy2, px / xy2, 0, 0,
            py * (vx * py - vy * px) / sqrtXy32, px * (vy * px - vx * py) / sqrtXy32,
            px / sqrtXy2, py / sqrtXy2;

    return Hj;
}