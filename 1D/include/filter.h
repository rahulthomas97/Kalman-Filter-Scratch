#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>

using mat2d = Eigen::Matrix2d;
using vec2d = Eigen::Vector2d;
class KalmanFilter
{
public:
    KalmanFilter(
        vec2d init_estimate, double dt, double u, mat2d A, vec2d B, vec2d C, mat2d Q, double R);

    void predict();            // Prediction step
    void update(double z);     // Update step
    vec2d getEstimate() const; // Get current estimate

private:
    vec2d estimate, C, B;
    mat2d A, Q, P, I;
    double u;
    double R;
};
