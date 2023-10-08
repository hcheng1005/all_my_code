#ifndef KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>

namespace TrackerDemo
{
    namespace SimpleTrack
    {

        // 定义系统状态
        // CA with shape :[X,Y,Z,]
        template <typename T>
        class State : public Kalman::Vector<T, 13>
        {
        public:
            KALMAN_VECTOR(State, T, 13)

            static constexpr size_t X = 0;
            static constexpr size_t Y = 1;
            static constexpr size_t Z = 2;
            static constexpr size_t VX = 3;
            static constexpr size_t VY = 4;
            static constexpr size_t VZ = 5;
            static constexpr size_t AX = 6;
            static constexpr size_t AY = 7;
            static constexpr size_t AZ = 8;
            static constexpr size_t LEN = 9;
            static constexpr size_t WID = 10;
            static constexpr size_t HEIGHT = 11;
            static constexpr size_t THETA = 12;

            T x() const { return (*this)[X]; }
            T y() const { return (*this)[Y]; }
            T z() const { return (*this)[Z]; }
            T vx() const { return (*this)[VX]; }
            T vy() const { return (*this)[VY]; }
            T vz() const { return (*this)[VZ]; }
            T ax() const { return (*this)[AX]; }
            T ay() const { return (*this)[AY]; }
            T az() const { return (*this)[AZ]; }
            T len() const { return (*this)[LEN]; }
            T wid() const { return (*this)[WID]; }
            T height() const { return (*this)[HEIGHT]; }
            T theta() const { return (*this)[THETA]; }

            T &x() { return (*this)[X]; }
            T &y() { return (*this)[Y]; }
            T &z() { return (*this)[Z]; }
            T &vx() { return (*this)[VX]; }
            T &vy() { return (*this)[VY]; }
            T &vz() { return (*this)[VZ]; }
            T &ax() { return (*this)[AX]; }
            T &ay() { return (*this)[AY]; }
            T &az() { return (*this)[AZ]; }
            T &len() { return (*this)[LEN]; }
            T &wid() { return (*this)[WID]; }
            T &height() { return (*this)[HEIGHT]; }
            T &theta() { return (*this)[THETA]; }
        };

        // NOT USED
        template <typename T>
        class Control : public Kalman::Vector<T, 2>
        {
        public:
            KALMAN_VECTOR(Control, T, 2)

            //! Velocity
            static constexpr size_t V = 0;
            //! Angular Rate (Orientation-change)
            static constexpr size_t DTHETA = 1;

            T v() const { return (*this)[V]; }
            T dtheta() const { return (*this)[DTHETA]; }

            T &v() { return (*this)[V]; }
            T &dtheta() { return (*this)[DTHETA]; }
        };

        template <typename T, template <class> class CovarianceBase = Kalman::StandardBase>
        class SystemModel_CAWithShape : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
        {
        public:
            //! State type shortcut definition
            typedef TrackerDemo::SimpleTrack::State<T> S;

            //! Control type shortcut definition
            typedef TrackerDemo::SimpleTrack::Control<T> C;

            // update cycle
            T dt = 0.1;

            // SystemModel_CAWithShape()
            // {
            //     this->F.setIdentity();
            //     this->F(0, 1) = dt;
            //     this->F(0, 2) = 0.5 * dt * dt;
            //     this->F(1, 2) = dt;

            //     this->F(3, 4) = dt;
            //     this->F(3, 5) = 0.5 * dt * dt;
            //     this->F(4, 5) = dt;

            //     this->F(6, 7) = dt;
            //     this->F(6, 8) = 0.5 * dt * dt;
            //     this->F(7, 8) = dt;
            // }

            S f(const S &x, const C &u) const
            {
                //! Predicted state vector after transition
                S x_ = x;

                x_.x() = x.x() + x.vx() * dt + 0.5 * x.ax() * dt * dt;
                x_.y() = x.y() + x.vy() * dt + 0.5 * x.ay() * dt * dt;
                x_.z() = x.y() + x.vy() * dt + 0.5 * x.ay() * dt * dt;
                x_.vx() = x.vx() + x.ax() * dt;
                x_.vy() = x.vy() + x.ay() * dt;
                x_.vz() = x.vz() + x.az() * dt;

                // Return transitioned state vector
                return x_;
            }

        protected:
            void updateJacobians(const S &x, const C &u)
            {
                this->F.setIdentity();

                this->F(0, 1) = dt;
                this->F(0, 2) = 0.5 * dt * dt;
                this->F(1, 2) = dt;

                this->F(3, 4) = dt;
                this->F(3, 5) = 0.5 * dt * dt;
                this->F(4, 5) = dt;

                this->F(6, 7) = dt;
                this->F(6, 8) = 0.5 * dt * dt;
                this->F(7, 8) = dt;

                this->W.setIdentity();
            }
        };

    } // namespace Radar
} // namespace KalmanExamples

#endif