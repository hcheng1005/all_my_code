#ifndef TARCEMANAGE_HPP_
#define TARCEMANAGE_HPP_

#include "PositionMeasurementModel.hpp"
#include "../../include/kalman/ExtendedKalmanFilter.hpp"
#include "SystemModel.hpp"

namespace TrackerDemo
{
    namespace SimpleTrack
    {
        template <typename T>
        class Trace
        {
        private:
            /* data */
        public:
            typedef TrackerDemo::SimpleTrack::State<T> S;
            typedef TrackerDemo::SimpleTrack::Control<T> C;
            typedef SimpleTrack::SystemModel_CAWithShape<T> SystemModel;
            typedef SimpleTrack::PositionMeasurementModel_3DBBox<T> MeasModel;
            typedef Kalman::ExtendedKalmanFilter<S> Filter;
            typedef PositionMeasurement_3DBBox<T> Meas;

            /* data */
            size_t ID;
            size_t Age;
            size_t matched_count = 0;
            size_t unmatched_coun = 0;

            Filter filter; // 定义滤波器
            SystemModel S_model;
            MeasModel M_model;

            Trace(const S &x, const size_t id)
            {
                filter.init(x); // 初始化
                ID = id;
                Age = 1;
            }

            void Predict(void)
            {
                filter.predict(S_model);
            }

            void Update(Meas meas)
            {
                filter.update(M_model, meas);
            }

            ~Trace() {}
        };
    }
}
#endif