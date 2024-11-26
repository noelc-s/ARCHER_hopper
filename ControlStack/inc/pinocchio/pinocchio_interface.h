#pragma once

#include "../Types.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <memory>

using namespace Hopper_t;

class PinocchioInterface
{
public:
    virtual ~PinocchioInterface() = default;

    /*! @brief  evaluate the forward dynamics
     *  @param [in] q  pos to evaluate the dynamics at
     *  @param [in] v  vel to evaluate the dynamics at
     *  @param [in] a  acc to evaluate the dynamics at
     *  @param [out] x_dot  the dynamics (dq, ddq)
     */
    virtual vector_t f(const vector_t &q, const vector_t &v, const vector_t &a, const domain &d) = 0;

    /*! @brief  compute the linearizations of f
     *  @param [in]     q, v, a  - state to compute the jacobians at
     *  @param [out]    A, B, C  - df/dx, df/du, and the residual
     */
    virtual void Df(const vector_t q, const vector_t v, const vector_t a, const domain d,
                    matrix_t &A, matrix_t &B, matrix_t &C, const vector_t q0) = 0;

    /*! @brief  compute the discrete dynamics at impact. calls impulse-dynamics
     *  @param [in]     q, v, J - state to compute the impact map at, and Jacobian of constraint
     *  @param [out]    x_plus   - the post impact state (q+, dq+)
     */
    virtual vector_t delta_f(const vector_t q, const vector_t v, const domain d) = 0;

    /*! @brief  compute the jacobian of the discrete dynamics at impact
     *  @param [in]     q, v, J - state to compute the impact map at, and Jacobian of constraint
     *  @param [out]    A, B, C   -  ddelta_f/dx, ddelta_f/du, and the residual
     */
    virtual void Ddelta_f(const vector_t q, const vector_t v, const domain d,
                          matrix_t &A, matrix_t &B, matrix_t &C, const vector_t q0) = 0;
};

// Factory function for creating the implementation
std::unique_ptr<PinocchioInterface> createPinocchioInstance();
