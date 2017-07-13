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
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            ekf_.initRadar(measurement_pack.raw_measurements_);
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            ekf_.initLidar(measurement_pack.raw_measurements_);
        }
        is_initialized_ = true;
        previous_timestamp_ = measurement_pack.timestamp_;
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

//    cout << "x_ = " << ekf_.x_ << endl;
//    cout << "P_ = " << ekf_.P_ << endl;
}

