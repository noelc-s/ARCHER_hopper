#pragma once

#include "../Types.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <memory>

using namespace Hopper_t;

class MPCInterface
{
public:
    virtual ~MPCInterface() = default;

    int nx, nu, nvar;
    // Parameters for the MPC program
    MPC_Parameters p;

    /*! @brief solve the mpc problem
     * @param [in] &sol the solution object to write
     * @param [in] &command the commanded global pos [x,y] and extra signal [flip, traj tracking, etc]
     * @param [in] &command_interp the interpolated command for trajectory tracking
     */
    virtual int solve(vector_t &sol, vector_3t &command, vector_2t &command_interp) = 0;

};

// Factory function for creating the implementation
std::unique_ptr<MPCInterface> createMPCInstance();
