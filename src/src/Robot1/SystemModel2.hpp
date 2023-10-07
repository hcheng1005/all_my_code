#ifndef KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>

namespace KalmanExamples{
namespace Radar{

/**
 * @brief System state vector-type for a 3DOF planar robot
 *
 * This is a system state for a very simple planar robot that
 * is characterized by its (x,y)-Position and angular orientation.
 *
 * @param T Numeric scalar type
 */
// CV:[X,Y,VX,VY]
template <typename T>
class State : public Kalman::Vector<T, 4>
{
public:
    KALMAN_VECTOR(State, T, 4)

    //! X-position
    static constexpr size_t X = 0;
    //! Y-Position
    static constexpr size_t Y = 1;
    //! VX
    static constexpr size_t VX = 2;
    //! VY
    static constexpr size_t VY = 3;

    T x() const { return (*this)[X]; }
    T y() const { return (*this)[Y]; }
    T vx() const { return (*this)[VX]; }
    T vy() const { return (*this)[VY]; }

    T &x() { return (*this)[X]; }
    T &y() { return (*this)[Y]; }
    T &vx() { return (*this)[VX]; }
    T &vy() { return (*this)[VY]; }
};

/**
 * @brief System control-input vector-type for a 3DOF planar robot
 *
 * This is the system control-input of a very simple planar robot that
 * can control the velocity in its current direction as well as the
 * change in direction.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(Control, T, 2)
    
    //! Velocity
    static constexpr size_t V = 0;
    //! Angular Rate (Orientation-change)
    static constexpr size_t DTHETA = 1;
    
    T v()       const { return (*this)[ V ]; }
    T dtheta()  const { return (*this)[ DTHETA ]; }
    
    T& v()      { return (*this)[ V ]; }
    T& dtheta() { return (*this)[ DTHETA ]; }
};


template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel_CV : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
	typedef KalmanExamples::Radar::State<T> S;
    
    //! Control type shortcut definition
    typedef KalmanExamples::Radar::Control<T> C;

    // update cycle
    T dt = 0.1;
    
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_;

        x_.x() = x.x() + x.vx() * dt; // + std::cos( newOrientation ) * u.v();
        x_.y() = x.y() + x.vy() * dt; // + std::sin( newOrientation ) * u.v();
        x_.vx() = x.vx();
        x_.vy() = x.vy();

        // Return transitioned state vector
        return x_;
    }
    
protected:
    void updateJacobians( const S& x, const C& u )
    {
        this->F.setIdentity();
        this->F(0, 1) = dt;
        this->F(2, 3) = dt;
        
        this->W.setIdentity();
    }
};

} // namespace Radar
} // namespace KalmanExamples

#endif