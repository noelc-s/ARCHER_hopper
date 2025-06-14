#include <cppad/cg.hpp>
#include "../inc/pinocchio/pinocchio_interface.h"

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

#include <chrono>
using namespace std::chrono;

class PinocchioWrapper : public PinocchioInterface
{
public:

    pinocchio::Model model;
    pinocchio::Data data;
    std::vector<RigidConstraintModelTpl<scalar_t, 0>> contact_model_ground;
    std::vector<RigidConstraintDataTpl<scalar_t, 0>> contact_data_ground;
    std::vector<RigidConstraintModelTpl<scalar_t, 0>> contact_model_flight;
    std::vector<RigidConstraintDataTpl<scalar_t, 0>> contact_data_flight;
    std::shared_ptr<CodeGenABAChild<double>> aba_code_gen;
    std::shared_ptr<CodeGenABADerivativesChild<double>> Daba_code_gen;
    constexpr scalar_t springStiffness = 1000;

    PinocchioWrapper::PinocchioWrapper()
    {
        // Construct Pinocchio model
        const std::string urdf_path_c = "../rsc/hopper.urdf";
        model = pinocchio::Model();
        pinocchio::urdf::buildModel(urdf_path_c, pinocchio::JointModelFreeFlyer(), model);
        data = Data(model);

        contact_model_ground.emplace_back(RigidConstraintModelTpl<scalar_t, 0>(CONTACT_3D, model, 2, SE3::Identity(), LOCAL_WORLD_ALIGNED));
        contact_data_ground.emplace_back(RigidConstraintDataTpl<scalar_t, 0>(contact_model_ground.at(0)));
        initConstraintDynamics(model, data, contact_model_ground);

        // Contact for when foot hits hard stops going to flight phase
        contact_model_flight.emplace_back(RigidConstraintModelTpl<scalar_t, 0>(CONTACT_3D, model, 2, LOCAL));
        contact_data_flight.emplace_back(RigidConstraintDataTpl<scalar_t, 0>(contact_model_flight.at(0)));
        initConstraintDynamics(model, data, contact_model_flight);

        aba_code_gen = std::make_shared<CodeGenABAChild<double>>(model);
        Daba_code_gen = std::make_shared<CodeGenABADerivativesChild<double>>(model);
        aba_code_gen->initLib();
        aba_code_gen->compileAndLoadLib(PINOCCHIO_CXX_COMPILER);
        Daba_code_gen->initLib();
        Daba_code_gen->compileAndLoadLib(PINOCCHIO_CXX_COMPILER);
    }

    vector_t PinocchioWrapper::f(const vector_t &q, const vector_t &v, const vector_t &a, const domain &d) override
    {
        vector_t x_dot(2 * model.nv);
        switch (d)
        {
        case flight:
        {
            // Not constrained dynamics for flight, because that would fix the foot position
            auto start = high_resolution_clock::now();
            aba(model, data, q, v, a);
            // aba_code_gen->evalFunction(q,v,a);
            auto end = high_resolution_clock::now();
            auto duration = duration_cast<nanoseconds>(end - start);
            std::cout << "    ABA: " << duration.count() * pow(10, -3) << " microseconds." << std::endl;
            // x_dot << v.segment(0, 6), 0, v.segment(7, 3), data.ddq;
            x_dot << v.segment(0, 6), 0, v.segment(7, 3), aba_code_gen->getVal();
            std::cout << "got val" << std::endl;
            break;
        }
        case ground:
        {
            initConstraintDynamics(model, data, contact_model_ground);
            constraintDynamics(model, data, q, v, a, contact_model_ground, contact_data_ground);
            vector_t springForce(10);
            springForce << 0, 0, springStiffness * q(7), 0, 0, 0, -springStiffness * q(7), 0, 0, 0;
            x_dot << v, data.ddq + springForce;
            break;
        }
        otherwise:
        {
            throw std::invalid_argument("Invalid domain in f");
            break;
        }
        }
        quat_t quat(q(6), q(3), q(4), q(5));
        x_dot.segment(0, 3) += cross(q.segment(0, 3)) * quat.inverse()._transformVector(v.segment(3, 3));
        return x_dot;
    };

    void PinocchioWrapper::Df(const vector_t q, const vector_t v, const vector_t a, const domain d,
                              matrix_t &A, matrix_t &B, matrix_t &C, const vector_t q0) override
    {
        matrix_t springJacobian(10, 10);
        // Call dynamics first to populate q, v, a, for ComputeConstraintDynamicsDerivatives
        vector_t f = PinocchioWrapper::f(q, v, a, d);
        switch (d)
        {
        case flight:
        {
            // Not constrained dynamics for flight, because that would fix the foot position
            auto start = high_resolution_clock::now();
            // computeABADerivatives(model, data, q, v, a);
            // Daba_code_gen->evalFunction(q,v,a);
            aba_code_gen->evalJacobian(q, v, a);
            auto end = high_resolution_clock::now();
            auto duration = duration_cast<nanoseconds>(end - start);
            std::cout << "    D_ABA: " << duration.count() * pow(10, -3) << " microseconds." << std::endl;

            springJacobian.setZero();

            A << matrix_t::Zero(10, 10), matrix_t::Identity(10, 10), aba_code_gen->da_dq + springJacobian, aba_code_gen->da_dv;

            matrix_t B_mat(10, 4);
            B_mat << 0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
            B << matrix_t::Zero(10, 4), data.Minv * B_mat;

            vector_t s(20);
            vector_t x(21);
            x << q, v;
            s = qk_to_xik(x, q0);

            C << f - A * s - B * a.tail(4);

            break;
        }
        case ground:
        {
            initConstraintDynamics(model, data, contact_model_ground);
            computeConstraintDynamicsDerivatives(model, data, contact_model_ground, contact_data_ground);
            springJacobian << matrix_t::Zero(2, 10),
                0, 0, 0, 0, 0, 0, springStiffness, 0, 0, 0,
                matrix_t::Zero(3, 10),
                0, 0, 0, 0, 0, 0, -springStiffness, 0, 0, 0,
                matrix_t::Zero(3, 10);
            A << matrix_t::Zero(10, 10), matrix_t::Identity(10, 10), data.ddq_dq + springJacobian, data.ddq_dv;

            matrix_t B_mat(10, 4);
            B_mat << 0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
            B << matrix_t::Zero(10, 4), data.Minv * B_mat;

            vector_t s(20);
            vector_t x(21);
            x << q, v;
            s = qk_to_xik(x, q0);

            C << f - A * s - B * a.tail(4);
            break;
        }
        otherwise:
        {
            throw std::invalid_argument("Invalid domain in Df");
            break;
        }
        }
    };

    vector_t PinocchioWrapper::delta_f(const vector_t q, const vector_t v, const domain d) override
    {
        const double r_coeff = 0;                   // restitution coeff -- assumes perfectly plastic
        const ProximalSettingsTpl<double> settings; // default has mu = 0
        switch (d)
        {
        case flight_ground:
        {
            initConstraintDynamics(model, data, contact_model_ground);
            impulseDynamics(model, data, q, v, contact_model_ground, contact_data_ground, r_coeff, settings);
            break;
        }
        case ground_flight:
        {
            initConstraintDynamics(model, data, contact_model_flight);
            impulseDynamics(model, data, q, v, contact_model_flight, contact_data_flight, r_coeff, settings);
            break;
        }
        }
        vector_t x_plus(21);
        x_plus << q, data.dq_after;
        return x_plus;
    };

    void PinocchioWrapper::Ddelta_f(const vector_t q, const vector_t v, const domain d,
                                    matrix_t &A, matrix_t &B, matrix_t &C, const vector_t q0) override
    {
        vector_t df(21);
        const double r_coeff = 0;                   // restitution coeff -- assumes perfectly plastic
        const ProximalSettingsTpl<double> settings; // default has mu = 0
        switch (d)
        {
        case flight_ground:
        {
            df = PinocchioWrapper::delta_f(q, v, d);
            initConstraintDynamics(model, data, contact_model_ground);
            computeImpulseDynamicsDerivatives(model, data, contact_model_ground, contact_data_ground, r_coeff, settings);
            break;
        }
        case ground_flight:
        {
            df = PinocchioWrapper::delta_f(q, v, d);
            initConstraintDynamics(model, data, contact_model_flight);
            computeImpulseDynamicsDerivatives(model, data, contact_model_flight, contact_data_flight, r_coeff, settings);
            break;
        }
        }
        A << matrix_t::Identity(10, 10), matrix_t::Zero(10, 10), data.ddq_dq, data.ddq_dv;

        B.setZero();

        vector_t s(20);
        vector_t x(21);
        x << q, v;
        s = qk_to_xik(x, q0);

        vector_t s_df(20);
        s_df << s.segment(0, 10), df.segment(11, 10);

        C << s_df - A * s;
    };

}

template <typename Scalar>
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

template <typename Scalar>
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

std::unique_ptr<PinocchioInterface> createPinocchioInstance()
{
    return std::make_unique<PinocchioWrapper>();
}