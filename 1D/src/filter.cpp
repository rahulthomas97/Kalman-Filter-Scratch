#include "filter.h"
#include <iostream>

KalmanFilter::KalmanFilter(
    vec2d init_estimate, double dt, double u, mat2d A, vec2d B, vec2d C, mat2d Q, double R)
    : estimate(init_estimate)
    , A(A)
    , B(B)
    , C(C)
    , Q(Q)
    , R(R)
{
    P = Eigen::Matrix2d::Identity();
    I = Eigen::Matrix2d::Identity();
}

void KalmanFilter::predict()
{
    estimate = A * estimate + B * u;
    P = A * P * A.transpose() + Q;
}

void KalmanFilter::update(double z)
{
    vec2d K = P * C * 1 / (C.transpose() * P * C + R);
    estimate = estimate + K * (z - C.transpose() * estimate);
    P = (I - K * C.transpose()) * P;
}

vec2d KalmanFilter::getEstimate() const
{
    return estimate;
}
