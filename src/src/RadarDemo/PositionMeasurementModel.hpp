#ifndef KALMAN_EXAMPLES_ROBOT1_POSITIONMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_ROBOT1_POSITIONMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include "SystemModel.hpp"
#include <iostream>

namespace TrackerDemo
{
    namespace SimpleTrack
    {
        /**
         * @brief Measurement vector measuring the robot position
         *
         * @param T Numeric scalar type
         */
        template <typename T>
        class PositionMeasurement_3DBBox : public Kalman::Vector<T, 7>
        {
        public:
            KALMAN_VECTOR(PositionMeasurement_3DBBox, T, 7)

            // 观测量定义
            static constexpr size_t X = 0;
            static constexpr size_t Y = 1;
            static constexpr size_t Z = 2;
            static constexpr size_t LEN = 3;
            static constexpr size_t WID = 4;
            static constexpr size_t HEIGHT = 5;
            static constexpr size_t THETA = 6;

            T x() const { return (*this)[X]; }
            T y() const { return (*this)[Y]; }
            T z() const { return (*this)[Z]; }
            T len() const { return (*this)[LEN]; }
            T wid() const { return (*this)[WID]; }
            T height() const { return (*this)[HEIGHT]; }
            T theta() const { return (*this)[THETA]; }

            T &x() { return (*this)[X]; }
            T &y() { return (*this)[Y]; }
            T &z() { return (*this)[Z]; }
            T &len() { return (*this)[LEN]; }
            T &wid() { return (*this)[WID]; }
            T &height() { return (*this)[HEIGHT]; }
            T &theta() { return (*this)[THETA]; }
        };

        template <typename T, template <class> class CovarianceBase = Kalman::StandardBase>
        class PositionMeasurementModel_3DBBox : public Kalman::LinearizedMeasurementModel<State<T>, PositionMeasurement_3DBBox<T>, CovarianceBase>
        {
        public:
            typedef TrackerDemo::SimpleTrack::State<T> S;
            typedef TrackerDemo::SimpleTrack::PositionMeasurement_3DBBox<T> M;

            M h(const S &x) const
            {
                M meas;

                meas.x() = x.x();
                meas.y() = x.y();
                meas.z() = x.z();
                meas.len() = x.len();
                meas.wid() = x.wid();
                meas.height() = x.height();
                meas.theta() = x.theta();

                return meas;
            }

        protected:
            // Kalman::Vector<T, 3> meas_;

        protected:
            /**
             * @names:
             * @description: from [x,y,vx,vy] --> [r,a,v]
             * @return {*}
             */
            void updateJacobians(const S &x)
            {
                this->H.setZero();
                this->H(x.X, 0) = 1.0;
                this->H(1, 1) = 1.0;
                this->H(2, 2) = 1.0;
                this->H(3, 9) = 1.0;
                this->H(4, 10) = 1.0;
                this->H(5, 11) = 1.0;
                this->H(6, 12) = 1.0;
            }
        };

    } // namespace Radar
} // namespace KalmanExamples

#endif