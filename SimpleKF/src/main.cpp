#include "filter.h"
#include "helper.h"
#include "matplotlibcpp.h"
#include <iostream>
#include <random>

#include <qt5/QtCore/QDebug>

namespace plt = matplotlibcpp;
using namespace std;
int main()
{
    TomlParser parser(CONFIG_DIR + std::string("/constAcc.toml"));
    double dt = parser.getValFromToml<double>("dt");
    cout<<dt<<endl;

    KalmanFilter kf(parser);

    std::vector<double> t, real_x;
    for (double i = 0; i <= 100; i += dt) {
        t.push_back(i);
        // Real values: real_x = 0.1 * ((t**2) - t)
        real_x.push_back(0.1 * (pow(i, 2) - i));
    }

    std::vector<double> predictions;
    std::vector<double> measurements;
    matd C = parser.getMatrixFromToml("C");
    std::random_device rd;  // Seed for the random number engine
    std::mt19937 gen(rd()); // Mersenne Twister engine

    // Create a normal distribution with specified mean and standard deviation
    std::normal_distribution<> dist(0, 50);

    // Loop through real_x to generate measurements and predictions
    for (double x : real_x) {
        vecd z;
        z.resize(1);
        z << C(0, 0) * x + dist(gen);

        measurements.push_back(z(0));
        kf.predict();
        vecd x_kf = kf.getEstimate();
        kf.update(z);

        predictions.push_back(x_kf(0));
    }
    plt::figure();
    plt::named_plot("measurements", t, measurements, "b-");
    plt::named_plot("ground truth", t, real_x, "y-");
    plt::named_plot("estimates", t, predictions, "r-");
    // // plt::hist(samples, 10);

    plt::legend();
    plt::show();

    return 0;
}
