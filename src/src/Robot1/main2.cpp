
// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>

#include "SystemModel2.hpp"
#include "PositionMeasurementModel2.hpp"

#include "../../include/kalman/ExtendedKalmanFilter.hpp"
#include "../../include/kalman/UnscentedKalmanFilter.hpp"

#include <iostream>
#include <random>
#include <chrono>
#include <fstream>

using namespace KalmanExamples;
using namespace std;

typedef float T;

// Some type shortcuts
typedef Radar::State<T> State;
typedef Radar::Control<T> Control;
typedef Radar::SystemModel_CV<T> SystemModel_CV_;

typedef Radar::PositionMeasurement_RAV<T> PositionMeasurement;
typedef Radar::PositionMeasurementModel_RAV<T> PositionModel;

int main(int argc, char **argv)
{
    string fname = "./src/src/Robot1/data.txt";
    std::ofstream csv_data(fname, ios::out);
    if (!csv_data.is_open())
    {
        cout << "Error: opening file fail" << endl;
    }

    // Simulated (true) system state
    State x;
    x << 1, 2, 3, 4;

    // Control input
    Control u;
    u.setZero();

    // System models
    SystemModel_CV_ sys;
    auto Q_ = sys.getCovariance();
    Q_ = Q_ * 0.1;
    sys.setCovariance(Q_);

    // Measurement models
    PositionModel pm(0, 0, 0);
    auto R_ = pm.getCovariance();
    R_ = R_ * 0.01;
    pm.setCovariance(R_);

    // Random number generation (for noise simulation)
    std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<T> noise(0, 1);

    // Extended Kalman Filter
    Kalman::ExtendedKalmanFilter<State> ekf;
    // Unscented Kalman Filter
    Kalman::UnscentedKalmanFilter<State> ukf(1);

    // Init filters with true system state
    ekf.init(x);
    ukf.init(x);

    // Standard-Deviation of noise added to all state vector components during state transition
    T systemNoise = 0.1;
    // Standard-Deviation of noise added to all measurement vector components in distance measurements
    T distanceNoise = 0.5;
    T angleNoise = 0.017 * 1;
    T velNoise = 0.2;

    // Simulate for 100 steps
    const size_t N = 100;
    for (size_t i = 1; i <= N; i++)
    {
        // Simulate system
        x = sys.f(x, u);

        auto meas_ = x;

        // Add noise: 
        meas_.x() += systemNoise * noise(generator);
        meas_.y() += systemNoise * noise(generator);
        meas_.vx() += systemNoise * noise(generator);
        meas_.vy() += systemNoise * noise(generator);

        // Predict state for current time-step using the filters
        auto x_ekf = ekf.predict(sys, u);
        auto x_ukf = ukf.predict(sys, u);

        // Position measurement
        // {
            // We can measure the position every 10th step
            PositionMeasurement position = pm.h(meas_);

            // Measurement is affected by noise as well
            position.r() += distanceNoise * noise(generator);
            position.a() += angleNoise * noise(generator);
            position.v() += velNoise * noise(generator);

            // Update EKF
            x_ekf = ekf.update(pm, position);

            // Update UKF
            x_ukf = ukf.update(pm, position);
        // }

        csv_data << x.x() << ", " << x.y() << ", " << x.vx() << ", " << x.vy() << ", " 
                    << position.r() * std::sin(position.a()) << ", " << position.r() * std::cos(position.a()) << ", " << meas_.vx() << ", " << meas_.vy() << ", " 
                    << x_ekf.x() << ", " << x_ekf.y() << ", " << x_ekf.vx() << ", " << x_ekf.vy() << ", "
                    << x_ukf.x() << ", " << x_ukf.y() << ", " << x_ukf.vx() << ", " << x_ukf.vy() << std::endl;
    }
    csv_data.close();
    return 0;
}
