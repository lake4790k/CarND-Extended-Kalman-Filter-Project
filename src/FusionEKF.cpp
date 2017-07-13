#include "FusionEKF.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF() : is_initialized_(false), previous_timestamp_(0) {
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    if (!is_initialized_) {
        initialize(measurement_pack);
        return;
    }

    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    ekf_.Predict(dt);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}

void FusionEKF::initialize(const MeasurementPackage &measurement_pack) {
    cout << "EKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        double rho = measurement_pack.raw_measurements_(0);
        double phi = measurement_pack.raw_measurements_(1);
        double rho_dot = measurement_pack.raw_measurements_(2);
        ekf_.x_(0) = rho * cos(phi);
        ekf_.x_(1) = rho * sin(phi);
        ekf_.x_(2) = rho_dot * cos(phi);
        ekf_.x_(3) = rho_dot * sin(phi);
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        const double px = measurement_pack.raw_measurements_(0);
        const double py = measurement_pack.raw_measurements_(1);
        ekf_.x_ << px, py, 0, 0;
    }

    cout << ekf_.x_ << endl;

    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
}

