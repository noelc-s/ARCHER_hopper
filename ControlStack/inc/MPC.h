#ifndef MPC_H
#define MPC_H

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
    matrix_t Ac, Bc, Cc, Ad, Bd, Cd; // matrices for the dynamics
    matrix_t Ac_, Bc_, Cc_, Ad_, Bd_, Cd_; // temprary variables

    Eigen::Matrix<domain, Eigen::Dynamic, 1> d_bar;
    vector_t elapsed_time;

    Eigen::SparseMatrix<double,ColMajor> dynamics_A;
    vector_t dynamics_b_lb, dynamics_b_ub;
    Eigen::SparseMatrix<double,ColMajor> SparseIdentity;
    Eigen::SparseMatrix<double,ColMajor> H;
    vector_t f, full_ref;

    OsqpEigen::Solver solver;

    // Parameters for the MPC program
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

    // Initialize the MPC object
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

    /*! @brief Estimate the time to impact of the hopper
    *
    * Neglects the orientation, and simply uses the nominal foot height minus the deflection of the spring.
    * Works as a first order estimate, but probably could be improvec.
    * @param[in] x Lie Group elements
    * @param[out] xi Lie Algebra elements
    */
    static scalar_t time2impact(vector_t x, scalar_t heightOffset);

    /*! @brief Take the state and apply the log of the orientation to get elements of the Lie Algebra
    * @param[in] x Lie Group elements
    * @param[out] xi Lie Algebra elements
    */
    static vector_t Log(vector_t x);

    /*! @brief Take elements of the Lie Algebra and Exp them to the Lie Group
    * @param[in] xi Lie Algebra elements
    * @param[out] x Lie Group elements
    */
    static vector_t Exp(vector_t xi);

    /*! @brief apply q0_inverse and then perform Log
    * @param[in] qk the Lie Group element
    * @param[in] q0 the base point
    * @param[out] xik the Lie Algebra element
    */
    static vector_t qk_to_xik(vector_t qk, vector_t q0);

    /*! @brief apply q0 and then perform Exp
    *
    * @param[in] xik the Lie Algebra element
    * @param[in] q0 the base point
    * @param[out] qk the Lie Group element
    */
    static vector_t xik_to_qk(vector_t xik, vector_t q0);

    /*! @brief Convert local frame to the global frame
    *
    * @param[in] x_l the local frame coordinate
    * @param[out] x_g the global frame coordinate
    */
    static vector_t local2global(vector_t x_l);

    /*! @brief Convert global frame to the local frame
    *
    * @param[in] x_g the global frame coordinate
    * @param[out] x_l the local frame coordinate
    */
    static vector_t global2local(vector_t x_g);

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
    void LinearizeDynamics(Hopper hopper, matrix_t x_bar, matrix_t u_bar, Eigen::Matrix<domain, Eigen::Dynamic, 1> d_bar, const vector_t q0, const vector_t elapsed_time);

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