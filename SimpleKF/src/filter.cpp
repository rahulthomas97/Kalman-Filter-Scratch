#include "filter.h"
#include <iostream>

KalmanFilter::KalmanFilter(
    vecd init_estimate, vecd u, matd A, matd B, matd C, matd Q, matd R, matd init_P)
    : estimate(init_estimate)
    , A(A)
    , B(B)
    , C(C)
    , Q(Q)
    , R(R)
    , u(u)
    , P(init_P)

{
    I = Eigen::MatrixXd::Identity(A.rows(), A.rows());
}

KalmanFilter::KalmanFilter(TomlParser parser)
{
    A = parser.getMatrixFromToml("A");
    B = parser.getMatrixFromToml("B");
    C = parser.getMatrixFromToml("C");
    Q = parser.getMatrixFromToml("Q");
    R = parser.getMatrixFromToml("R");
    P = parser.getMatrixFromToml("init_P");
    estimate = parser.getVectorFromToml("init_estimate");
    I = Eigen::MatrixXd::Identity(A.rows(), A.rows());
    u = parser.getVectorFromToml("u");
}

void KalmanFilter::predict()
{
    estimate = A * estimate + B * u;
    P = A * P * A.transpose() + Q;
}

void KalmanFilter::update(vecd z)
{
    vecd K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    estimate = estimate + K * (z - C * estimate);
    P = (I - K * C) * P;
}

vecd KalmanFilter::getEstimate() const
{
    return estimate;
}
