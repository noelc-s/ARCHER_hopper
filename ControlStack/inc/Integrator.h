#include "Hopper.h"
#include "utils.h"

class Integrator {
public:
  const scalar_t dt;
  const matrix_t Q;
  const matrix_t R;

  Integrator(scalar_t dt, matrix_t Q, matrix_t R) : dt(dt), Q(Q), R(R) {};

  void populate_data(std::shared_ptr<Hopper> hopper, const vector_t &q_k, 
                        const vector_t &v_k, const vector_t &a, const domain &d, vector_ht &x_dot, matrix_t &A, matrix_t &B, matrix_t &C);

  void x_integrator(std::shared_ptr<Hopper> hopper, const matrix_t &A, const vector_t &x_dot, const vector_t& q_k,
                        const vector_t &v_k, const vector_t& a, vector_t &x_kp1);

  void p_integrator(const matrix_t &A, const vector_t& x_k, 
                const vector_t &p_k, const vector_t& a, vector_t &p_kp1);

  void J_integrator(const scalar_t J_k, const vector_t &x_k, const vector_t &u_k, scalar_t &J_kp1);

  void xpj_integrator(std::shared_ptr<Hopper> hopper, const vector_t &x_k, const vector_t p_k, const scalar_t J_k, const vector_4t &u_k, 
                        const domain &d, vector_t &x_kp1, vector_t &p_kp1, scalar_t &J_kp1);
};