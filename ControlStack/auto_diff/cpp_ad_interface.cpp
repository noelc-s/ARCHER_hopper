//
// Created by zolkin on 8/21/24.
//

#include "cpp_ad_interface.h"

#include <memory>

namespace torc::ad {
    CppADInterface::CppADInterface(ad_fcn_t cg_fn,
            std::string identifier, fs::path deriv_path,
            DerivativeOrder deriv_order,
            int x_size, int p_size,
            const bool& force_compile, std::vector<std::string> compile_flags)
            : name_(std::move(identifier)), deriv_path_(std::move(deriv_path)), compile_flags_(std::move(compile_flags)),
              user_function_(std::move(cg_fn)), x_size_(x_size), p_size_(p_size), deriv_order_(std::move(deriv_order)) {

        lib_path_no_ext_ = deriv_path_ / (name_ + "_deriv_lib");
        lib_name_  = name_ + "_deriv_lib" + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;
        lib_path_ = deriv_path_ / lib_name_;

        if (!fs::exists(deriv_path_)) {
            std::cout << "[CppADInterface] Creating a directory for the derivative libraries at " << deriv_path_ << std::endl;
            fs::create_directory(deriv_path_);
        }

        if (force_compile || !fs::exists(lib_path_)) {
            std::cout << "[CppADInterface] Compiling model..." << std::endl;
            CreateModel();
        } else {
            std::cout << "[CppADInterface] Loading model from " << lib_path_ << std::endl;
            LoadModel();
        }
    }

    void CppADInterface::GetFunctionValue(const torc::ad::vectorx_t& x, const torc::ad::vectorx_t& p,
                                          torc::ad::vectorx_t& y) const {
        if (p.size() != p_size_) {
            throw std::runtime_error("[CppADInterface] " + name_ + " Invalid parameter size! Parameter size: " + std::to_string(p.size()) + " expected: " + std::to_string(p_size_));
        }

        if (x.size() != x_size_) {
            throw std::runtime_error("[CppADInterface] " + name_ + " Invalid domain size!");
        }

        vectorx_t xp_combined(x_size_ + p_size_);
        xp_combined << x, p;
        ad_model_->ForwardZero(xp_combined, y);
    }

    void CppADInterface::GetJacobian(const torc::ad::vectorx_t& x, const torc::ad::vectorx_t& p,
                                     torc::ad::matrixx_t& jacobian) const {
         if (x.size() != x_size_) {
            throw std::runtime_error("[CppADInterface] " + name_ + " Invalid domain size!");
         }

        if (p.size() != p_size_) {
            throw std::runtime_error("[CppADInterface] " + name_ + " Invalid parameter size! Parameter size: " + std::to_string(p.size()) + " expected: " + std::to_string(p_size_));
        }

        vectorx_t xp_combined(x_size_ + p_size_);
        xp_combined << x, p;
        CppAD::cg::ArrayView<double> xp_array_view(xp_combined.data(), xp_combined.size());

        std::vector<double> sparse_jac(nnz_jac_);
        CppAD::cg::ArrayView<double> sparse_jac_array_view(sparse_jac);
        size_t const* rows;
        size_t const* cols;
        // From the comment in OCS2:
        // Call this particular SparseJacobian. Other CppAd functions allocate internal vectors that are incompatible with multithreading.
        ad_model_->SparseJacobian(xp_array_view, sparse_jac_array_view, &rows, &cols);

        // Write sparse elements into Eigen type. Only jacobian w.r.t. variables was requested, so cols should not contain elements corresponding
        // to parameters.
        jacobian.setZero(ad_model_->Range(), x_size_);
        for (size_t i = 0; i < nnz_jac_; i++) {
            jacobian(rows[i], cols[i]) = sparse_jac[i];
        }

        assert(jacobian.allFinite());
    }

    void CppADInterface::GetGaussNewton(const torc::ad::vectorx_t& x, const torc::ad::vectorx_t& p,
                                        torc::ad::matrixx_t& jacobian,
                                        torc::ad::matrixx_t& hessian) const {
        GetJacobian(x, p, jacobian);

        // TODO: Might consider doing this more efficiently
        hessian = jacobian.transpose() * jacobian;
    }

    void CppADInterface::GetHessian(const torc::ad::vectorx_t& x, const torc::ad::vectorx_t& p, const vectorx_t& w,
                                    torc::ad::matrixx_t& hessian) const {
        if (deriv_order_ != SecondOrder) {
            throw std::runtime_error("[CppADInterface] Attempting to get the Hessian when SecondOrder is not specified!");
        }

        // Concatenate input
        vectorx_t xp_combined(x_size_ + p_size_);
        xp_combined << x, p;
        CppAD::cg::ArrayView<const double> xp_array_view(xp_combined.data(), xp_combined.size());

        std::vector<double> sparse_hess(nnz_hess_);
        CppAD::cg::ArrayView<double> sparse_hess_array_view(sparse_hess);
        size_t const* rows;
        size_t const* cols;

        if (w.size() != y_size_) {
            std::cerr << "w size: " << w.size() << std::endl;
            std::cerr << "y size: " << y_size_ << std::endl;
            throw std::runtime_error("[CppADInterface] hessian weight vector of the wrong size!");
        }

        CppAD::cg::ArrayView<const double> w_array_view(w.data(), w.size());

        // Call this particular SparseHessian. Other CppAd functions allocate internal vectors that are incompatible with multithreading.
        ad_model_->SparseHessian(xp_array_view, w_array_view, sparse_hess_array_view, &rows, &cols);

        // Fills upper triangular sparsity of hessian w.r.t variables.
        hessian.setZero(x_size_, x_size_);
        for (size_t i = 0; i < nnz_hess_; i++) {
            hessian(rows[i], cols[i]) = sparse_hess[i];
        }

        // Copy upper triangular to lower triangular part
        hessian.template triangularView<Eigen::StrictlyLower>() = hessian.template triangularView<Eigen::StrictlyUpper>().transpose();

        assert(hessian.allFinite());
    }

    void CppADInterface::GetHessian(int idx, const torc::ad::vectorx_t& x, const torc::ad::vectorx_t& p,
                                    torc::ad::matrixx_t& hessian) const {
        vectorx_t w = vectorx_t::Zero(y_size_);
        w(idx) = 1;
        GetHessian(x, p, w, hessian);
    }

    // ----- Get Sparsity Patterns ----- //
    void CppADInterface::GetJacobianSparsityPatternMat(matrixx_t& J) const {
        J = jac_sparsity_mat_;
    }

    void CppADInterface::GetHessianSparsityPatternMat(matrixx_t& H) const {
        H = hess_sparsity_mat_;
    }

    void CppADInterface::GetGaussNewtonSparsityPatternMat(matrixx_t& H) const {
        H = gauss_newton_sparsity_mat_;
    }

    sparsity_pattern_t CppADInterface::GetJacobianSparsityPatternSet() const {
        if (ad_model_->isJacobianSparsityAvailable()) {
            return jac_sparsity_set_;
        } else {
            throw std::runtime_error("[CppADInterface] Cannot get Jacobian sparsity pattern!");
        }
    }

    sparsity_pattern_t CppADInterface::GetGaussNewtonSparsityPatternSet() const {
        if (ad_model_->isJacobianSparsityAvailable()) {
            return gauss_newton_sparsity_set_;
        } else {
            throw std::runtime_error("[CppADInterface] Cannot get Gauss Newton sparsity pattern!");
        }
    }

    sparsity_pattern_t CppADInterface::GetHessianSparsityPatternSet() const {
        if (ad_model_->isHessianSparsityAvailable()) {
            return hess_sparsity_set_;
        } else {
            throw std::runtime_error("[CppADInterface] Cannot get Hessian sparsity pattern!");
        }
    }

    // ----- Get Sizes ----- //
    int CppADInterface::GetDomainSize() const {
        return x_size_;
    }

    int CppADInterface::GetRangeSize() const {
        return y_size_;
    }

    int CppADInterface::GetParameterSize() const {
        return p_size_;
    }

    sparsity_pattern_t CppADInterface::GetSparsityPatternCols(const sparsity_pattern_t& sparsity, int start_col, int num_cols) {
        sparsity_pattern_t sparsity_set;
        sparsity_set.resize(sparsity.size());
        for (int row = 0; row < sparsity.size(); row++) {
            for (const auto& col : sparsity[row]) {
                if (col >= start_col && col < start_col + num_cols) {
                    sparsity_set[row].insert(col - start_col);
                }
            }
        }

        return sparsity_set;
    }


    // ----- Library Path ----- //
    fs::path CppADInterface::GetLibPath() const {
        return lib_path_;
    }

    // ----- Model Creation and Loading ----- //
    void CppADInterface::CreateModel() {
        // Need one vector to make independent
        ad_vector_t xp_combined = ad_vector_t(x_size_ + p_size_);
        xp_combined.setOnes();
        CppAD::Independent(xp_combined);

        // Dependent variables
        ad_vector_t y;

        // Tape the function
        user_function_(xp_combined.head(x_size_), xp_combined.tail(p_size_), y);

        // Create autodiff function object
        AD::ADFun<cg_t> ad_fn(xp_combined, y);
        ad_fn.optimize();

        y_size_ = y.size();

        // generate library source code
        ADCG::ModelCSourceGen<double> c_gen(ad_fn, this->name_);

        if (deriv_order_ == SecondOrder) {
            c_gen.setCreateSparseHessian(true);
            c_gen.setCustomSparseHessianElements(GetHessianSparsity(ad_fn));
        }

        c_gen.setCreateSparseJacobian(true);
        c_gen.setCustomSparseJacobianElements(GetJacobianSparsity(ad_fn));

        ADCG::ModelLibraryCSourceGen<double> lib_gen(c_gen);
        ADCG::DynamicModelLibraryProcessor<double> lib_processor(lib_gen, lib_path_no_ext_);

        CppAD::cg::GccCompiler<double> gcc_compiler;
        gcc_compiler.setCompileFlags(compile_flags_);

        std::cout << "Here" << std::endl;

        // TODO: Do I need to save the sources?

        // here is the problem

        // compile source code into a dynamic library
        dynamic_lib_ = lib_processor.createDynamicLibrary(gcc_compiler);
        std::cout << "Here" << std::endl;
        ad_model_ = dynamic_lib_->model(this->name_);
        std::cout << "Here" << std::endl;

        SetNonZeros();
        std::cout << "Here" << std::endl;
        UpdateJacobianSparsityPattern();
        std::cout << "Here" << std::endl;
        UpdateHessianSparsityPattern();
        std::cout << "Here" << std::endl;
        UpdateGaussNewtonSparsityPattern();
        std::cout << "Here" << std::endl;
    }

    void CppADInterface::LoadModel() {
        // throw std::runtime_error("[CppADInterface] Loading models is currently unsupported!");
        dynamic_lib_ = std::make_unique<CppAD::cg::LinuxDynamicLib<double>>(lib_path_.string());
        if (dynamic_lib_ == nullptr) {
            throw std::runtime_error("[CppADInterface] Could not load the derivative library!");
        }

        ad_model_.reset();
        ad_model_ = dynamic_lib_->model(this->name_);
        if (ad_model_ == nullptr) {
            throw std::runtime_error("[CppADInterface] Could not retrieve the derivative model from the library!");
        }

        y_size_ = ad_model_->Range();

        SetNonZeros();
        UpdateJacobianSparsityPattern();
        UpdateHessianSparsityPattern();
        UpdateGaussNewtonSparsityPattern();
    }

    // ----- Sparsity ----- //
    void CppADInterface::SetNonZeros() {
        if (ad_model_->isJacobianSparsityAvailable()) {
            nnz_jac_ = NumNonZeros(ad_model_->JacobianSparsitySet());
        }
        if (ad_model_->isHessianSparsityAvailable()) {
            nnz_hess_ = NumNonZeros(ad_model_->HessianSparsitySet());
        }
    }

    size_t CppADInterface::NumNonZeros(const torc::ad::sparsity_pattern_t& sparsity) {
        size_t nnz = 0;
        for (const auto& row : sparsity) {
            nnz += row.size();
        }
        return nnz;
    }

    torc::ad::sparsity_pattern_t CppADInterface::GetJacobianSparsity(AD::ADFun<cg_t>& ad_fn) const {
        auto jac_sparsity = CppAD::cg::jacobianSparsitySet<sparsity_pattern_t>(ad_fn);
        sparsity_pattern_t x_vars(y_size_);
        for (auto& sparsity_row : x_vars) {
            for (size_t i = 0; i < x_size_; i++) {
                sparsity_row.insert(i);
            }
        }

        // Now grab the intersection
        return GetIntersection(jac_sparsity, x_vars);
    }

    torc::ad::sparsity_pattern_t CppADInterface::GetHessianSparsity(AD::ADFun<cg_t>& ad_fn) {
        auto hess_sparsity = CppAD::cg::hessianSparsitySet<sparsity_pattern_t>(ad_fn);
        sparsity_pattern_t x_vars(x_size_ + p_size_);
        for (size_t i = 0; i < x_size_; i++) {
            for (size_t j = 0; j < x_size_; j++) {
                x_vars[i].insert(j);
            }
        }

        // Now grab the intersection
        return GetIntersection(hess_sparsity, x_vars);
    }

    torc::ad::sparsity_pattern_t CppADInterface::GetIntersection(const sparsity_pattern_t& sp_1, const sparsity_pattern_t& sp_2) {
        const auto numRows = sp_1.size();

        sparsity_pattern_t result(sp_1.size());
        for (int row = 0; row < numRows; row++) {
            std::set_intersection(sp_1[row].begin(), sp_1[row].end(), sp_2[row].begin(),
                                  sp_2[row].end(), std::inserter(result[row], result[row].begin()));
        }
        return result;
    }

    torc::ad::sparsity_pattern_t CppADInterface::GetUnion(const sparsity_pattern_t& sp_1, const sparsity_pattern_t& sp_2) {
        const int num_rows = std::min(sp_1.size(), sp_2.size());

        sparsity_pattern_t result(num_rows);
        for (int row = 0; row < num_rows; row++) {
            for (const auto& col : sp_1[row]) {
                result[row].insert(col);
            }

            for (const auto& col : sp_2[row]) {
                result[row].insert(col);
            }
        }

        return result;
    }

    void CppADInterface::UpdateJacobianSparsityPattern() {
        if (ad_model_->isJacobianSparsityAvailable()) {
            // Update for the matrix
            auto jac_sparsity_set = ad_model_->JacobianSparsitySet();
            jac_sparsity_mat_.resize(y_size_, x_size_);
            jac_sparsity_mat_.setZero();
            for (int row = 0; row < jac_sparsity_mat_.rows(); row++) {
                for (int col = 0; col < jac_sparsity_mat_.cols(); col++) {
                    if (jac_sparsity_set[row].contains(col)) {
                        jac_sparsity_mat_(row, col) = 1;
                    }
                }
            }

            // Update for the set
            sparsity_pattern_t x_vars(y_size_);
            for (auto& sparsity_row : x_vars) {
                for (size_t i = 0; i < x_size_; i++) {
                    sparsity_row.insert(i);
                }
            }

            // Now grab the intersection
            jac_sparsity_set_ = GetIntersection(jac_sparsity_set, x_vars);
        }
    }

    void CppADInterface::UpdateHessianSparsityPattern() {
        if (ad_model_->isHessianSparsityAvailable()) {
            // Update for the matrix
            auto hess_sparsity_set = ad_model_->HessianSparsitySet();
            hess_sparsity_mat_.resize(y_size_, x_size_);    // TODO: Is this size correct?
            hess_sparsity_mat_.setZero();
            for (int row = 0; row < hess_sparsity_mat_.rows(); row++) {
                for (int col = 0; col < hess_sparsity_mat_.cols(); col++) {
                    if (hess_sparsity_set[row].contains(col)) {
                        hess_sparsity_mat_(row, col) = 1;
                    }
                }
            }

            // Update for the set
            sparsity_pattern_t x_vars(x_size_ + p_size_);
            for (size_t i = 0; i < x_size_; i++) {
                for (size_t j = 0; j < x_size_; j++) {
                    x_vars[i].insert(j);
                }
            }

            // Now grab the intersection
            hess_sparsity_set_ = GetIntersection(hess_sparsity_set, x_vars);

            // if (hess_sparsity_set.size() != x_size_) {
            //     std::cerr << "Hess sparsity set size: " << hess_sparsity_set.size() << std::endl;
            //     std::cerr << "Domain size: " << x_size_ << std::endl;
            //     throw std::logic_error("[CppADInterface] Generated hessian pattern sizes don't match!");
            // }
        }
    }

    void CppADInterface::UpdateGaussNewtonSparsityPattern() {
        // Matrix
        gauss_newton_sparsity_mat_ = jac_sparsity_mat_.transpose() * jac_sparsity_mat_;
        for (int row = 0; row < gauss_newton_sparsity_mat_.rows(); row++) {
            for (int col = 0; col < gauss_newton_sparsity_mat_.cols(); col++) {
                if (gauss_newton_sparsity_mat_(row, col) != 0) {
                    gauss_newton_sparsity_mat_(row, col) = 1;
                }
            }
        }

        // Set
        gauss_newton_sparsity_set_.resize(gauss_newton_sparsity_mat_.rows());
        for (int row = 0; row < gauss_newton_sparsity_mat_.rows(); row++) {
            for (int col = 0; col < gauss_newton_sparsity_mat_.cols(); col++) {
                if (gauss_newton_sparsity_mat_(row, col) != 0) {
                    gauss_newton_sparsity_set_[row].insert(col);
                }
            }
        }
    }

}   // namespace torc::ad