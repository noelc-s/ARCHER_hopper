#include <Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h"
#include <iostream>
#include "Types.h"
#include "yaml-cpp/yaml.h"
#include "Hopper.h"
#include <manif/manif.h>

using namespace Hopper_t;
using namespace Eigen;

class MPC {
public:
    int nx, nu, nvar;
    matrix_t Ac;
    matrix_t Bc;
    matrix_t Cc;
    matrix_t Ad;
    matrix_t Bd;
    matrix_t Cd;

    // temprary variables
    matrix_t Ac_;
    matrix_t Bc_;
    matrix_t Cc_;
    matrix_t Ad_;
    matrix_t Bd_;
    matrix_t Cd_;

    Eigen::Matrix<domain, Eigen::Dynamic, 1> d_bar;
    vector_t elapsed_time;

    Eigen::SparseMatrix<double,ColMajor> dynamics_A;
    vector_t dynamics_b_lb, dynamics_b_ub;
    Eigen::SparseMatrix<double,ColMajor> SparseIdentity;
    Eigen::SparseMatrix<double,ColMajor> H;
    vector_t f;
    vector_t full_ref;

    OsqpEigen::Solver solver;

    struct MPC_Params {
        int N;
	int SQP_iter;
        vector_t stateScaling;
        vector_t inputScaling;
        scalar_t discountFactor;
	scalar_t dt_flight;
	scalar_t dt_ground;
	scalar_t tau_max;
	scalar_t f_max;
	scalar_t terminalScaling;
	scalar_t groundDuration;
	scalar_t heightOffset;
	scalar_t time_between_contacts;
	scalar_t hop_height;
	scalar_t circle_freq;
	scalar_t circle_amp;
	scalar_t max_vel;
    } p;

    MPC(int nx, int nu, MPC_Params &loaded_p) {
        this->nx = nx;
        this->nu = nu;
	p = loaded_p;

        std::cout << "Horizon length: " << p.N << std::endl;
        std::cout << "Pos gain: " << p.stateScaling(0) << ", " << p.stateScaling(1) << std::endl;
        std::cout << "Vel gain: " << p.stateScaling(10) << ", " << p.stateScaling(11) << std::endl;

	d_bar.resize(p.N-1,1);
	elapsed_time.resize(p.N,1);

        nvar = nx*p.N+nu*(p.N-1);

        Ac.resize(nx,nx*(p.N-1));
        Bc.resize(nx,nu*(p.N-1));
        Cc.resize(nx,p.N-1);
        Ad.resize(nx,nx*(p.N-1));
        Bd.resize(nx,nu*(p.N-1));
        Cd.resize(nx,p.N-1);

        // temprary variables
        Ac_.resize(nx,nx);
        Bc_.resize(nx,nu);
        Cc_.resize(nx,1);
        Ad_.resize(nx,nx);
        Bd_.resize(nx,nu);
        Cd_.resize(nx,1);

        dynamics_A.resize(nx*p.N+(p.N-1)*4,(nx*p.N+nu*(p.N-1)));
        SparseIdentity.resize(nx*p.N+(p.N-1)*4,(nx*p.N+nu*(p.N-1)));
        dynamics_b_lb.resize(nx*p.N+(p.N-1)*4);
        dynamics_b_ub.resize(nx*p.N+(p.N-1)*4);

	f.resize(nx*p.N + nu*(p.N-1));
	full_ref.resize(nx*p.N + nu*(p.N-1));
	H.resize(nx*p.N + nu*(p.N-1),nx*p.N + nu*(p.N-1));

        solver.settings()->setWarmStart(true);
	solver.settings()->setVerbosity(false);
	//solver.settings()->setAbsoluteTolerance(1e-9);
        solver.data()->setNumberOfVariables(nx*p.N + nu*(p.N-1));
	solver.data()->setNumberOfConstraints(nx*p.N+(p.N-1)*4);
//        solver.data()->setNumberOfConstraints();

        reset();
        buildCost();
	buildDynamicEquality();
	solver.data()->setHessianMatrix(H);
        solver.data()->setGradient(f);
        solver.data()->setLinearConstraintsMatrix(dynamics_A);
        solver.data()->setLowerBound(dynamics_b_lb);
        solver.data()->setUpperBound(dynamics_b_ub);
	// instantiate the solver
        solver.initSolver();
    }

    static scalar_t time2impact(vector_t x, scalar_t heightOffset);
    static vector_t local2global(vector_t x_l);
    static vector_t global2local(vector_t x_g);
    static vector_t Log(vector_t x);
    static vector_t Exp(vector_t x);
    static vector_t qk_to_xik(vector_t qk, vector_t q0);
    static vector_t xik_to_qk(vector_t xik, vector_t q0);

    vector_t oneStepPredict(Hopper hopper, const vector_t xi, const vector_t tau, const float dt, const domain d, const vector_t q0);

    void LinearizeDynamics(Hopper hopper, matrix_t x_bar, matrix_t u_bar, Eigen::Matrix<domain, Eigen::Dynamic, 1> d_bar, const vector_t q0, const vector_t elapsed_time);

    void reset();

    int solve(Hopper hopper, vector_t &sol, vector_3t &command, vector_2t &command_interp);

    void buildDynamicEquality();
    void updateDynamicEquality(vector_t x0);

    void buildCost();
};
