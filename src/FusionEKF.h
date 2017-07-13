#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include "Eigen/Dense"

#include "measurement_package.h"
#include "kalman_filter.h"

class FusionEKF {
public:
    FusionEKF();

    virtual ~FusionEKF();

    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    KalmanFilter ekf_;

private:
    bool is_initialized_;

    long long previous_timestamp_;


};

#endif
