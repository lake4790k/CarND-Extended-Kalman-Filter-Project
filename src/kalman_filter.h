#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;

class KalmanFilter {
public:

    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transition matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;

    KalmanFilter();

    virtual ~KalmanFilter();

    void initRadar(const VectorXd &z);

    void initLidar(const VectorXd &z);

    void Predict(const double dt);

    void Update(const Eigen::VectorXd &z);

    void UpdateEKF(const Eigen::VectorXd &z);

private:

    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_laser_;

    double lastDt_;

    void updateF(const double dt);
    void updateQ(const double dt);

    void updateXandP(const VectorXd &z, const VectorXd &z_pred);

    MatrixXd CalculateJacobian(const VectorXd& x_state);

};

#endif
