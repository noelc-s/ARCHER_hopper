//
// Created by zolkin on 8/21/24.
//

#ifndef AUTO_DIFF_TYPES_H
#define AUTO_DIFF_TYPES_H

#include <Eigen/Core>

#include <cppad/cg.hpp>

namespace torc::ad {
    namespace ADCG = CppAD::cg;
    namespace AD = CppAD;

    using vectorx_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;
    using matrixx_t = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
    using cg_t = ADCG::CG<double>;
    using adcg_t = CppAD::AD<cg_t>;

    using sparsity_pattern_t = std::vector<std::set<size_t>>;
    using ad_vector_t = Eigen::Matrix<adcg_t, Eigen::Dynamic, 1>;
    using ad_matrix_t = Eigen::Matrix<adcg_t, Eigen::Dynamic, Eigen::Dynamic>;

    using ad_fcn_t = std::function<void(const ad_vector_t&, const ad_vector_t&, ad_vector_t&)>;
}   // namepsace torc::ad

#endif //AUTO_DIFF_TYPES_H
