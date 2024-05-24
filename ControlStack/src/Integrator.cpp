#include "../inc/Integrator.h"

void Integrator::populate_data(std::shared_ptr<Hopper> hopper, const vector_t &q_k, 
              const vector_t &v_k, const vector_t &a, const domain &d, 
              vector_ht &x_dot, matrix_t &A, matrix_t &B, matrix_t &C) {
  x_dot = hopper->f(q_k, v_k, a, d);
  hopper->Df(q_k, v_k, a, d, A, B, C, q_k);
}

void Integrator::x_integrator(std::shared_ptr<Hopper> hopper, const matrix_t &A, const vector_t &x_dot, 
              const vector_t& q_k, const vector_t &v_k, const vector_t& a, vector_t &x_kp1) {
vector_t q_kp1(11);
vector_t v_kp1(10);
// v_kp1 << v_k + dt*(M - dt*A.block(10,10,10,10)).ldlt().solve(M*x_dot.segment<10>(10));
v_kp1 << v_k + dt*(matrix_t::Identity(10,10) - dt*hopper->data.Minv*A.block(10,10,10,10)).ldlt().solve(x_dot.segment<10>(10));
hopper->discrete_step(hopper->model, q_k, v_kp1, q_kp1, dt);
x_kp1 << q_kp1, v_kp1;
}

void Integrator::p_integrator(const matrix_t &A, const vector_t& x_k, 
              const vector_t &p_k, const vector_t& a, vector_t &p_kp1) {
  vector_t Dcost(21);
  Dcost << Q*x_k;
  p_kp1 << p_k + dt*(Dcost.head(20) + A.transpose()*p_k);
}

void Integrator::J_integrator(const scalar_t J_k, const vector_t &x_k, const vector_t &u_k, scalar_t &J_kp1) {
  J_kp1 = J_k + dt*((scalar_t)(x_k.transpose()*Q*x_k) + (scalar_t)(u_k.transpose()*R*u_k));
}

void Integrator::xpj_integrator(std::shared_ptr<Hopper> hopper, const vector_t &x_k, const vector_t p_k, const scalar_t J_k, const vector_4t &u_k, 
                              const domain &d, vector_t &x_kp1, vector_t &p_kp1, scalar_t &J_kp1) {
  matrix_t A(20, 20);
  matrix_t B(20, 4);
  matrix_t C(20, 1);
  vector_t a(10);
  vector_t q_k(11);
  vector_t v_k(10);
  a << 0, 0, 0, 0, 0, 0, u_k;
  q_k = x_k.segment<11>(0);
  v_k = x_k.segment<10>(11);
  vector_ht x_dot;
  populate_data(hopper, q_k, v_k, a, d, x_dot, A, B, C);
  switch (d)
  {
  case flight:
  case ground:
    x_integrator(hopper, A, x_dot, q_k, v_k, a, x_kp1);
    p_integrator(A, x_k, p_k, a, p_kp1);
    J_integrator(J_k, x_k, u_k, J_kp1);
    break;
  case flight_ground:
  case ground_flight:
    x_kp1 = hopper->delta_f(q_k, v_k, d);
    p_kp1 = A.transpose().ldlt().solve(p_k);
    J_kp1 = J_k;
    break;
  }
}
