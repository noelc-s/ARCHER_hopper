#include <gtest/gtest.h>
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include <cppad/cg.hpp>
#include "pinocchio/codegen/cppadcg.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <filesystem>

#include "cpp_ad_interface.h"
#include "pinocchio/codegen/code-generator-algo.hpp"

#include <chrono>
using namespace std::chrono;

using namespace pinocchio;
using namespace Eigen;
namespace fs = std::filesystem;

namespace ADCG = CppAD::cg;
namespace AD = CppAD;

using vectorx_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using matrixx_t = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using cg_t = ADCG::CG<double>;
using adcg_t = CppAD::AD<cg_t>;
using ad_vector_t = Eigen::Matrix<adcg_t, Eigen::Dynamic, 1>;
using ad_matrix_t = Eigen::Matrix<adcg_t, Eigen::Dynamic, Eigen::Dynamic>;
using ad_fcn_t = std::function<void(const ad_vector_t&, const ad_vector_t&, ad_vector_t&)>;
using ad_pin_model_t = pinocchio::ModelTpl<adcg_t>;
using ad_pin_data_t = pinocchio::DataTpl<adcg_t>;


// Example function in your project
int add(int a, int b) {
    return a + b;
}

void Aba(const ad_pin_model_t& model, const std::shared_ptr<ad_pin_data_t> data, const ad_vector_t& qva, const ad_vector_t& p, ad_vector_t& f) {

    ad_vector_t q(11);
    ad_vector_t v(10);
    ad_vector_t a(10);

    q << qva.segment(0,11);
    v << qva.segment(11,10);
    a << qva.segment(21,10);

    // Compute error
    f = aba(model, *data, q, v, a);
    // f = integrate(model, q, v);
}

// Test case for add function
TEST(AdditionTest, HandlesPositiveInput) {
    EXPECT_EQ(add(1, 2), 3);
    EXPECT_EQ(add(10, 20), 30);
}

TEST(AdditionTest, HandlesNegativeInput) {
    EXPECT_EQ(add(-1, -1), -2);
    EXPECT_EQ(add(-1, 1), 0);
}

TEST(Timing, LoadModel) {
    // Construct Pinocchio model
    const std::string urdf_path_c = "../rsc/hopper.urdf";
    Model model = pinocchio::Model();
    pinocchio::urdf::buildModel(urdf_path_c, pinocchio::JointModelFreeFlyer(), model);
    Data data = Data(model);

    VectorXd q(11);
    VectorXd v(10);
    VectorXd tau(10);

    Quaternion quat = Quaternion<double>(1,0,0,0);
    q << 0, 0, 1, quat.coeffs(), 0, 0, 0, 0;
    v << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    tau << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    for (int i = 0; i < 10; i++) {
        auto start = high_resolution_clock::now();
        aba(model, data, q, v, tau);
        auto end = high_resolution_clock::now();
        auto duration = duration_cast<nanoseconds>(end - start);
        std::cout << "    ABA: " << duration.count() * pow(10, -3) << " microseconds." << std::endl;
    }

    for (int i = 0; i < 10; i++) {
        auto start = high_resolution_clock::now();
        computeABADerivatives(model, data, q, v, tau);
        auto end = high_resolution_clock::now();
        auto duration = duration_cast<nanoseconds>(end - start);
        std::cout << "    D_ABA: " << duration.count() * pow(10, -3) << " microseconds." << std::endl;
    }

    // bool compile_derivatives = true;

    // ad_pin_model_t pin_ad_model = model.cast<adcg_t>();
    // std::shared_ptr<ad_pin_data_t> pin_ad_data = std::make_shared<ad_pin_data_t>(pin_ad_model);

    // ad_fcn_t function_wrapper = [pin_ad_model, pin_ad_data](const ad_vector_t& qva, const ad_vector_t& p, ad_vector_t& f) {
    //     Aba(pin_ad_model, pin_ad_data, qva, p, f);
    // };


    // auto aba_func = std::make_unique<torc::ad::CppADInterface>(
    //         function_wrapper,
    //         "aba",
    //         "cppad",
    //         torc::ad::DerivativeOrder::FirstOrder, 31, 1,
    //         compile_derivatives
    //     );

    CodeGenABA<double> aba_code_gen(model);
    aba_code_gen.initLib();
    aba_code_gen.compileAndLoadLib(PINOCCHIO_CXX_COMPILER);

    for (int i = 0; i < 10; i++) {
        auto start = high_resolution_clock::now();
        aba_code_gen.evalFunction(q, v, tau);
        auto end = high_resolution_clock::now();
        auto duration = duration_cast<nanoseconds>(end - start);
        std::cout << "    CG ABA: " << duration.count() * pow(10, -3) << " microseconds." << std::endl;
    }

    for (int i = 0; i < 10; i++) {
        auto start = high_resolution_clock::now();
        aba_code_gen.evalJacobian(q, v, tau);
        auto end = high_resolution_clock::now();
        auto duration = duration_cast<nanoseconds>(end - start);
        std::cout << "    CG D_ABA: " << duration.count() * pow(10, -3) << " microseconds." << std::endl;
    }
    // aba_code_gen.evalJacobian(q, v, tau);
    // aba_code_gen.da_dq;
    // aba_code_gen.da_dv;
    // aba_code_gen.da_dtau;

    // std::cout << "Done compiling" << std::endl;

    // VectorXd x(31);
    // x << q, v, tau;
    // VectorXd p(1);
    // VectorXd y(10);
    // MatrixXd jac(10,31);
    // p.setZero();
    // y.setZero();
    // jac.setZero();

    // aba_func->GetFunctionValue(x,p,y);
    // aba_func->GetJacobian(x,p,jac);

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}