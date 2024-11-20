//
// Created by zolkin on 8/21/24.
//

#ifndef CPP_AD_INTERFACE_H
#define CPP_AD_INTERFACE_H

#include <functional>
#include <string>
#include <filesystem>

#include "auto_diff_types.h"
#include "pinocchio/codegen/cppadcg.hpp" // TODO: Why do I need to have this to make the pinocchio autodiff stuff work?


namespace torc::ad {
    namespace fs = std::filesystem;

    enum DerivativeOrder {
        FirstOrder,
        SecondOrder
    };

    // Heavily inspired by the OCS2 toolbox
    class CppADInterface {
    public:
        CppADInterface(ad_fcn_t cg_fn,
                       std::string identifier, fs::path deriv_path,
                       DerivativeOrder deriv_order,
                       int x_size, int p_size,
                       const bool& force_compile = false,
                       std::vector<std::string> compile_flags = {"-O3", "-march=native", "-shared", "-rdynamic"});

        void GetFunctionValue(const vectorx_t& x, const vectorx_t& p, vectorx_t& y) const;
        void GetJacobian(const vectorx_t& x, const vectorx_t& p, matrixx_t& jacobian) const;
        void GetGaussNewton(const vectorx_t& x, const vectorx_t& p, matrixx_t& jacobian, matrixx_t& hessian) const;

        /**
         * @brief returns the sum of the hessians for each term. If you only want the hessian of term, then set all the
         * other weights to 0.
         * @param x
         * @param p
         * @param w
         * @param hessian
         */
        void GetHessian(const vectorx_t& x, const vectorx_t& p, const vectorx_t& w, matrixx_t& hessian) const;

        void GetHessian(int idx, const vectorx_t& x, const vectorx_t& p, matrixx_t& hessian) const;

        // ----- Matrix Based Sparsity Pattern ----- //
        void GetJacobianSparsityPatternMat(matrixx_t& J) const;
        void GetGaussNewtonSparsityPatternMat(matrixx_t& H) const;
        void GetHessianSparsityPatternMat(matrixx_t& H) const;

        // ----- Set Based Sparsity Pattern ----- //
        // TODO: Make this a return by reference!
        [[nodiscard]] sparsity_pattern_t GetJacobianSparsityPatternSet() const;
        [[nodiscard]] sparsity_pattern_t GetGaussNewtonSparsityPatternSet() const;
        [[nodiscard]] sparsity_pattern_t GetHessianSparsityPatternSet() const;


        [[nodiscard]] fs::path GetLibPath() const;

        [[nodiscard]] int GetRangeSize() const;
        [[nodiscard]] int GetDomainSize() const;
        [[nodiscard]] int GetParameterSize() const;

        static sparsity_pattern_t GetSparsityPatternCols(const sparsity_pattern_t& sparsity, int start_col, int last_col);

        static sparsity_pattern_t GetIntersection(const sparsity_pattern_t& sp_1, const sparsity_pattern_t& sp_2);
        static sparsity_pattern_t GetUnion(const sparsity_pattern_t& sp_1, const sparsity_pattern_t& sp_2);

    protected:
        void CreateModel();

        void LoadModel();

        void SetNonZeros();

        // ----- Sparsity ----- //
        static size_t NumNonZeros(const sparsity_pattern_t& sparsity);

        sparsity_pattern_t GetJacobianSparsity(AD::ADFun<cg_t>& ad_fn) const;
        void UpdateJacobianSparsityPattern();

        sparsity_pattern_t GetHessianSparsity(AD::ADFun<cg_t>& ad_fn);
        void UpdateHessianSparsityPattern();


        void UpdateGaussNewtonSparsityPattern();

    private:
    std::string name_;
    fs::path lib_path_;
    std::string lib_name_;
    std::string lib_path_no_ext_;

    int x_size_;
    int p_size_;
    int y_size_;

    // Code gen compiler flags
    std::vector<std::string> compile_flags_;
    fs::path deriv_path_;

    // Autodiff-able function provided by the user
    ad_fcn_t user_function_;

    // How many derivatives to use
    DerivativeOrder deriv_order_;

    // Code gen objects
    std::unique_ptr<CppAD::cg::DynamicLib<double>> dynamic_lib_;
    std::unique_ptr<CppAD::cg::GenericModel<double>> ad_model_;

    // Sparsity information
    size_t nnz_jac_;
    size_t nnz_hess_;

    matrixx_t jac_sparsity_mat_;
    matrixx_t gauss_newton_sparsity_mat_;
    matrixx_t hess_sparsity_mat_;

    sparsity_pattern_t jac_sparsity_set_;
    sparsity_pattern_t gauss_newton_sparsity_set_;
    sparsity_pattern_t hess_sparsity_set_;
    };
}   // namepsace torc::ad


#endif //CPP_AD_INTERFACE_H
