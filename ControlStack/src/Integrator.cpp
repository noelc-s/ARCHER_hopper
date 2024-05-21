


const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

class Integrator {
public:
  const scalar_t dt;

  Integrator(scalar_t dt) : dt(dt) {};

  void populate_data(const pinocchio::Model &model, pinocchio::Data &data, const vector_t &q_k, const vector_t &v_k, const vector_t &a, vector_ht &x_dot, matrix_t &A, matrix_t &B, matrix_t &M) {
    f(model, data, q_k, v_k, a, x_dot);
    M = data.M;
    Df(model, data, q_k, v_k, a, A, B);
  }

  void x_integrator(const pinocchio::Model model, const matrix_t &A, const vector_t &x_dot, const vector_t& q_k, const vector_t &v_k, const matrix_t &M, const vector_t& a, vector_t &x_kp1) {
  vector_t q_kp1(11);
  vector_t v_kp1(10);
  v_kp1 << v_k + dt*(M - dt*A.block(10,10,10,10)).ldlt().solve(M*x_dot.segment<10>(10));
  discrete_step(model, q_k, v_kp1, q_kp1, dt);
  x_kp1 << q_kp1, v_kp1;
  }

  void p_integrator(const matrix_t &Q, const matrix_t &A, const vector_t& x_k, const vector_t &p_k, const vector_t& a, vector_t &p_kp1) {
    vector_t Dcost(21);
    Dcost << Q*x_k;
    p_kp1 << p_k + dt*(Dcost.head(20) + A.transpose()*p_k);
  }

  void J_integrator(const matrix_t &Q, const matrix_t &R, const scalar_t J_k, const vector_t &x_k, const vector_t &u_k, scalar_t &J_kp1) {
    J_kp1 = J_k + dt*((scalar_t)(x_k.transpose()*Q*x_k) + (scalar_t)(u_k.transpose()*R*u_k));
  }

  void xpj_integrator(const pinocchio::Model &model, pinocchio::Data data, const matrix_t &Q, const matrix_t &R, const vector_t &x_k, const vector_t p_k, const scalar_t J_k, const vector_3t &u_k, vector_t &x_kp1, vector_t &p_kp1, scalar_t &J_kp1) {
    matrix_t A(20,20);
    matrix_t M(10,10);
    matrix_t B(20,4);
    vector_t a(10);
    vector_t q_k(11);
    vector_t v_k(10);
    a << 0,0,0,0,0,0,u_k,0;
    q_k = x_k.segment<11>(0);
    v_k = x_k.segment<10>(11);
    vector_ht x_dot;
    populate_data(model, data, q_k, v_k, a, x_dot, A, B, M);
    x_integrator(model, A, x_dot, q_k, v_k, M, a, x_kp1);
    p_integrator(Q, A, x_k, p_k, a, p_kp1);
    J_integrator(Q, R, J_k, x_k, u_k, J_kp1);
  }
};