#include <cppad/cg.hpp>
#include "Types.h"
#include "utils.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/codegen/code-generator-algo.hpp"

#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/constrained-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/impulse-dynamics.hpp"
#include "pinocchio/algorithm/impulse-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"

using namespace Hopper_t;
using namespace pinocchio;

template<typename Scalar>
class CodeGenABAChild : public pinocchio::CodeGenABA<Scalar>
{
public:
    using Base = pinocchio::CodeGenABA<Scalar>;
    using typename Base::MatrixXs;

    // Constructor that forwards to the base class constructor
    CodeGenABAChild(
        const typename Base::Model &model,
        const std::string &function_name = "aba",
        const std::string &library_name = "cg_aba_eval")
        : Base(model, function_name, library_name)
    {
    }

    // Public getter methods
    MatrixXs getVal() const { return this->res; }
};

template<typename Scalar>
class CodeGenABADerivativesChild : public pinocchio::CodeGenABADerivatives<Scalar>
{
public:
    using Base = pinocchio::CodeGenABADerivatives<Scalar>;
    using typename Base::MatrixXs;

    // Constructor that forwards to the base class constructor
    CodeGenABADerivativesChild(
        const typename Base::Model &model,
        const std::string &function_name = "partial_aba",
        const std::string &library_name = "cg_partial_aba_eval")
        : Base(model, function_name, library_name)
    {
    }

    // Public getter methods
    MatrixXs getDddqDq() const { return this->dddq_dq; }
    MatrixXs getDddqDv() const { return this->dddq_dv; }
    MatrixXs getDddqDtau() const { return this->dddq_dtau; }
};

class PinocchioWrapper {
    public: 
        PinocchioWrapper();

    /*! @brief  evaluate the forward dynamics
    *  @param [in] q  pos to evaluate the dynamics at
    *  @param [in] v  vel to evaluate the dynamics at
    *  @param [in] a  acc to evaluate the dynamics at
    *  @param [out] x_dot  the dynamics (dq, ddq)
    */
    vector_t f(const vector_t& q, const vector_t& v, const vector_t& a, const domain& d);

    /*! @brief  compute the linearizations of f
    *  @param [in]     q, v, a  - state to compute the jacobians at
    *  @param [out]    A, B, C  - df/dx, df/du, and the residual
    */
    void Df(const vector_t q, const vector_t v, const vector_t a, const domain d,
            matrix_t &A, matrix_t &B, matrix_t &C, const vector_t q0);

    /*! @brief  compute the discrete dynamics at impact. calls impulse-dynamics
    *  @param [in]     q, v, J - state to compute the impact map at, and Jacobian of constraint
    *  @param [out]    x_plus   - the post impact state (q+, dq+)
    */
    vector_t delta_f(const vector_t q, const vector_t v, const domain d);

    /*! @brief  compute the jacobian of the discrete dynamics at impact
    *  @param [in]     q, v, J - state to compute the impact map at, and Jacobian of constraint
    *  @param [out]    A, B, C   -  ddelta_f/dx, ddelta_f/du, and the residual
    */
    void Ddelta_f(const vector_t q, const vector_t v, const domain d,
                  matrix_t &A, matrix_t &B, matrix_t &C, const vector_t q0);

    pinocchio::Model model;
    pinocchio::Data data;
    std::vector<RigidConstraintModelTpl<scalar_t, 0>> contact_model_ground;
    std::vector<RigidConstraintDataTpl<scalar_t, 0>> contact_data_ground;
    std::vector<RigidConstraintModelTpl<scalar_t, 0>> contact_model_flight;
    std::vector<RigidConstraintDataTpl<scalar_t, 0>> contact_data_flight;

    std::shared_ptr<CodeGenABAChild<double>> aba_code_gen;
    std::shared_ptr<CodeGenABADerivativesChild<double>> Daba_code_gen;
    scalar_t springStiffness;
};