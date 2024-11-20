#ifndef MPC_H
#define MPC_H

#include <Eigen/Dense>
#include "osqp++.h"
#include <iostream>
#include "Types.h"
#include "yaml-cpp/yaml.h"
#include "Hopper.h"
#include <manif/manif.h>
#include "utils.h"

using namespace Hopper_t;
using namespace Eigen;
using namespace osqp;

class MPC {
public:
    int nx, nu, nvar;
    matrix_t Ac, Bc, Cc, Ad, Bd, Cd; // matrices for the dynamics
    matrix_t Ac_, Bc_, Cc_, Ad_, Bd_, Cd_; // temprary variables

    Eigen::Matrix<domain, Eigen::Dynamic, 1> d_bar;
    vector_t elapsed_time;

    Eigen::SparseMatrix<double,ColMajor> dynamics_A;
    vector_t dynamics_b_lb, dynamics_b_ub;
    Eigen::SparseMatrix<double,ColMajor> SparseIdentity;
    Eigen::SparseMatrix<double,ColMajor> H;
    vector_t f, full_ref;

    OsqpSolver solver;
    OsqpInstance instance;
    OsqpSettings settings;


    // Parameters for the MPC program
    MPC_Parameters p;

    // Initialize the MPC object
    MPC(int nx, int nu, MPC_Parameters &loaded_p) {
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

    settings.verbose = false;
    settings.polish = true;
    settings.warm_start = true;
    settings.max_iter = 50;

    reset();
    buildCost();
	buildDynamicEquality();
    instance.objective_matrix = H;
    instance.objective_vector.resize(nvar);
    instance.objective_vector << f;
    instance.constraint_matrix = dynamics_A;
    instance.lower_bounds.resize(dynamics_b_lb.size());
    instance.upper_bounds.resize(dynamics_b_ub.size());
    instance.lower_bounds << dynamics_b_lb;
    instance.upper_bounds << dynamics_b_ub;
    // instantiate the solver
    auto status = solver.Init(instance, settings);
    }

    /*! @brief Estimate the time to impact of the hopper
    *
    * Neglects the orientation, and simply uses the nominal foot height minus the deflection of the spring.
    * Works as a first order estimate, but probably could be improvec.
    * @param[in] x Lie Group elements
    * @param[out] xi Lie Algebra elements
    */
    static scalar_t time2impact(vector_t x, scalar_t heightOffset);

    /*! @brief Apply the discrete time dynamics to predict where the system will be in on dt time step
    *
    * @param[in] hopper the hopping robot object
    * @param[in] xi the Lie Algebra elements
    * @param[in] tau the control input
    * @param[in] dt discretization time
    * @param[in] domain current domain of the system
    * @param[in] q0 base point for the exp operation
    * @param[out] x_kp1 vector of the predicted states one dt in the future
    */
    vector_t oneStepPredict(Hopper hopper, const vector_t xi, const vector_t tau, const float dt, const domain d, const vector_t q0);

    /*! @brief Apply the discrete time dynamics to predict where the system will be in on dt time step
    * @param[in] hopper the hopping robot object
    * @param[in] x_bar states to linearize around
    * @param[in] u_bar input to linearize around
    * @param[in] d_bar vector of domains over the horizon
    * @param[in] q0 base point for the exp operation
    * @param[in] elapsed_time the time when the linearization should take place (used for dt caluclation)
    * @param[(implicit) out] Ac
    * @param[(implicit) out] Bc
    * @param[(implicit) out] Cc
    * @param[(implicit) out] Ad
    * @param[(implicit) out] Bd
    * @param[(implicit) out] Cd
    */
    void LinearizeDynamics(Hopper& hopper, matrix_t x_bar, matrix_t u_bar, Eigen::Matrix<domain, Eigen::Dynamic, 1> d_bar, const vector_t q0, const vector_t elapsed_time);

    /*! @brief Reset all of the internal variables*/
    void reset();

    /*! @brief Set the equality constraints for the MPC program*/
    void buildDynamicEquality();

    /*! @brief Update the equality constraints
    *  ... must call LinearizeDynamics first to update internal states
    * @param[(implicit) in] Ad
    * @param[(implicit) in] Bd
    * @param[(implicit) in] Cd
    * @param[in] x0 vector of initial conditions
    */
    void updateDynamicEquality(vector_t x0);

    /*! @brief solve the mpc problem
     * @param [in] hopper the hopping robot object
     * @param [in] &sol the solution object to write
     * @param [in] &command the commanded global pos [x,y] and extra signal [flip, traj tracking, etc]
     * @param [in] &command_interp the interpolated command for trajectory tracking
     */
    int solve(Hopper hopper, vector_t &sol, vector_3t &command, vector_2t &command_interp);

    /*! @brief build the cost function matrices*/
    void buildCost();
};

#endif