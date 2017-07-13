#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() : lastDt_(-1) {
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);

    R_laser_ << 0.0225, 0,
            0, 0.0225;

    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    x_ = VectorXd(4);

    F_ = MatrixXd(4, 4);
    F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Q_ = MatrixXd(4, 4);

    P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::initRadar(const VectorXd &z) {
    double rho = z(0);
    double phi = z(1);
    double px = rho * cos(phi);
    double py = rho * sin(phi);
    // Although radar gives velocity data in the form of the range rate
    // a radar measurement does not contain enough information
    // to determine the state variable velocities
    x_ << px, py, 0, 0;
}

void KalmanFilter::initLidar(const VectorXd &z) {
    const double px = z(0);
    const double py = z(1);
    x_ << px, py, 0, 0;
}


void KalmanFilter::Predict(const double dt) {
    if (fabs(dt - lastDt_) > 0.00001) {
        updateF(dt);
        updateQ(dt);
        lastDt_ = dt;
    }

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    H_ = H_laser_;
    VectorXd z_pred = H_ * x_;
    R_ = R_laser_;
    updateXandP(z, z_pred);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    const double px = x_(0);
    const double py = x_(1);
    const double vx = x_(2);
    const double vy = x_(3);

    const double rho = sqrt(px * px + py * py);
    const double phi = atan2(py, px);
    double rho_dot = 0.;
    if (fabs(rho) > 0.0001) {
        rho_dot = (px * vx + py * vy) / rho;
    }
    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;

    R_ = R_radar_;
    H_ = tools.CalculateJacobian(x_);

    updateXandP(z, z_pred);
}

void KalmanFilter::updateXandP(const VectorXd &z, const VectorXd &z_pred) {
    VectorXd y = z - z_pred;
    while (y(1) < -M_PI) {
        y(1) += 2 * M_PI;
    }
    while (y(1) > M_PI) {
        y(1) -= 2 * M_PI;
    }
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::updateF(const double dt) {
    F_(0, 2) = dt;
    F_(1, 3) = dt;
}

void KalmanFilter::updateQ(const double dt) {
    double noise_ax = 9;
    double noise_ay = 9;

    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    Q_ << dt4 / 4 * noise_ax, 0, dt3 / 2 * noise_ax, 0,
            0, dt4 / 4 * noise_ay, 0, dt3 / 2 * noise_ay,
            dt3 / 2 * noise_ax, 0, dt2 * noise_ax, 0,
            0, dt3 / 2 * noise_ay, 0, dt2 * noise_ay;
}
