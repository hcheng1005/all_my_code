#ifndef KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>

namespace KalmanExamples
{
namespace Radar
{

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

/**
 * @brief System model for a simple planar 3DOF robot
 *
 * This is the system model defining how our robot moves from one 
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
	typedef KalmanExamples::Radar::State<T> S;
    
    //! Control type shortcut definition
    typedef KalmanExamples::Radar::Control<T> C;
    
    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to 
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
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
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */
    void updateJacobians( const S& x, const C& u )
    {
        // F = df/dx (Jacobian of state transition w.r.t. the state)
        this->F.setZero();
        
        // partial derivative of x.x() w.r.t. x.x()
        this->F( S::X, S::X ) = 1;
        // partial derivative of x.x() w.r.t. x.theta()
        this->F( S::X, S::THETA ) = -std::sin( x.theta() + u.dtheta() ) * u.v();
        
        // partial derivative of x.y() w.r.t. x.y()
        this->F( S::Y, S::Y ) = 1;
        // partial derivative of x.y() w.r.t. x.theta()
        this->F( S::Y, S::THETA ) = std::cos( x.theta() + u.dtheta() ) * u.v();
        
        // partial derivative of x.theta() w.r.t. x.theta()
        this->F( S::THETA, S::THETA ) = 1;
        
        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setIdentity();
        // TODO: more sophisticated noise modelling
        //       i.e. The noise affects the the direction in which we move as 
        //       well as the velocity (i.e. the distance we move)
    }
};

} // namespace Radar
} // namespace KalmanExamples

#endif