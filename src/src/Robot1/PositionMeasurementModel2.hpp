#ifndef KALMAN_EXAMPLES_ROBOT1_POSITIONMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_ROBOT1_POSITIONMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include <iostream>

namespace KalmanExamples{
namespace Radar{
/**
 * @brief Measurement vector measuring the robot position
 *
 * @param T Numeric scalar type
 */
template <typename T>
class PositionMeasurement_RAV : public Kalman::Vector<T, 3>
{
public:
    KALMAN_VECTOR(PositionMeasurement_RAV, T, 3)

    // 观测量定义
    static constexpr size_t R = 0;
    static constexpr size_t A = 1;
    static constexpr size_t V = 2;

    T r() const { return (*this)[R]; }
    T a() const { return (*this)[A]; }
    T v() const { return (*this)[V]; }

    T &r() { return (*this)[R]; }
    T &a() { return (*this)[A]; }
    T &v() { return (*this)[V]; }
};

template <typename T, template <class> class CovarianceBase = Kalman::StandardBase>
class PositionMeasurementModel_RAV : public Kalman::LinearizedMeasurementModel<State<T>, PositionMeasurement_RAV<T>, CovarianceBase>
{
public:
    //! State type shortcut definition : [x, y, vx, vy]
    typedef KalmanExamples::Radar::State<T> S;

    //! Measurement type shortcut definition: [R, A, V]
    typedef KalmanExamples::Radar::PositionMeasurement_RAV<T> M;

    PositionMeasurementModel_RAV(T r, T a, T v)
    {
        // init 
        meas_ << r, a, v;

        this->V.setIdentity();
    }

    M h(const S &x) const
    {
        M measurement;

        measurement.r() = std::sqrt(std::pow(x.x(), 2) + std::pow(x.y(), 2));
        measurement.a() = std::atan2(x.x(), x.y());
        measurement.v() = (x.x() * x.vx() + x.y() * x.vy()) / measurement.r();

        return measurement;
    }

protected:
    Kalman::Vector<T, 3> meas_;

protected:

    /**
     * @names: 
     * @description: from [x,y,vx,vy] --> [r,a,v]
     * @return {*}
     */
    void updateJacobians(const S &x)
    {
        // // H = dh/dx (Jacobian of measurement function w.r.t. the state)
        this->H.setZero();

        T range_ = std::sqrt(std::pow(x.x(), 2.0) + std::pow(x.y(), 2.0));
        this->H(0,0) = x.x() / range_;
        this->H(0,1) = x.y() / range_;
        this->H(1,0) = x.y() / (range_ * range_);
        this->H(1,1) = -x.x() / (range_ * range_);
        this->H(2,0) = x.vx() / range_ - x.x() *(x.x()*x.vx()+x.y()*x.vy()) / std::pow(range_, 3.0);
        this->H(2,1) = x.vy() / range_ - x.y() *(x.x()*x.vx()+x.y()*x.vy()) / std::pow(range_, 3.0);
        this->H(2,2) = x.x() / range_;
        this->H(2,3) = x.y() / range_;
    }
};

} // namespace Radar
} // namespace KalmanExamples

#endif