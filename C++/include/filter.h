#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <helper.h>

using matd = Eigen::MatrixXd;
using vecd = Eigen::VectorXd;
class KalmanFilter
{
public:
    KalmanFilter(vecd init_estimate, vecd u, matd A, matd B, matd C, matd Q, matd R, matd init_P);
    KalmanFilter(TomlParser);
    void predict();           // Prediction step
    void update(vecd z);      // Update step
    vecd getEstimate() const; // Get current estimate

private:
    vecd estimate, u;
    matd A, Q, P, I, C, B, R;
};
