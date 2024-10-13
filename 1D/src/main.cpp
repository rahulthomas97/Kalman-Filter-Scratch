#include "filter.h"
#include "matplotlibcpp.h"
#include <iostream>
#include <random>

namespace plt = matplotlibcpp;
int main()
{
    vec2d init_estimate(0, 0);
    double dt = 0.1;
    std::vector<double> t, real_x;
    for (double i = 0; i <= 100; i += dt) {
        t.push_back(i);
        // Real values: real_x = 0.1 * ((t**2) - t)
        real_x.push_back(0.1 * (pow(i, 2) - i));
    }

    double u = 1;
    double sigma_a = 0.25;
    double sigma_z = 2.0;
    double R = pow(sigma_z, 2);
    vec2d B, C;
    mat2d A, Q;
    A << 1, dt, 0, 1;
    Q << pow(dt, 4) / 4, pow(dt, 3) / 2, pow(dt, 3) / 2, pow(dt, 2);
    Q *= pow(sigma_a, 2);
    C = {1, 0};

    KalmanFilter kf(init_estimate, dt, u, A, B, C, Q, R);

    // Random number generator for noise
    std::default_random_engine generator;
    std::normal_distribution<double> noise_dist(0.0, 50.0);

    std::vector<double> predictions;
    std::vector<double> measurements;

    // Loop through real_x to generate measurements and predictions
    for (double x : real_x) {
        double z = C[0] * x + noise_dist(generator); // Simulate measurement with noise

        measurements.push_back(z);
        kf.predict();
        vec2d x_kf = kf.getEstimate();
        kf.update(z);

        predictions.push_back(x_kf(0));
    }
    plt::figure();
    plt::named_plot("measurements", t, measurements, "b-");
    plt::named_plot("ground truth", t, real_x, "y-");
    plt::named_plot("estimates", t, predictions, "r-");

    plt::legend();
    plt::show();

    return 0;
}
