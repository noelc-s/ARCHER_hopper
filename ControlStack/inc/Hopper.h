# pragma once
//
// Created by igor on 7/20/22.
//

#ifndef HOPPER_HOPPER_H
#define HOPPER_HOPPER_H

#include <iostream>
#include "Types.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include "yaml-cpp/yaml.h"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/contact-info.hpp"

using namespace Hopper_t;
using namespace pinocchio;

enum domain {flight, ground, flight_ground, ground_flight};

class Hopper {

public:
    scalar_t t;
    vector_3t pos;
    quat_t quat;
    vector_3t vel;
    vector_3t omega;
  
    bool contact;
    scalar_t num_hops;  

    scalar_t last_impact_time;
    scalar_t last_flight_time;
    scalar_t leg_pos;
    scalar_t leg_vel;
    vector_3t wheel_vel;
    vector_t q;
    vector_t v;
    domain dom;
    scalar_t multiplier_on_deltaf;

    vector_4t torque;

    pinocchio::Model model;
    pinocchio::Data data;
    std::vector<RigidConstraintModelTpl<scalar_t, 0>> contact_model_ground;
    std::vector<RigidConstraintDataTpl<scalar_t, 0>> contact_data_ground;
    std::vector<RigidConstraintModelTpl<scalar_t, 0>> contact_model_flight;
    std::vector<RigidConstraintDataTpl<scalar_t, 0>> contact_data_flight;

    quat_t quat_actuator;

    struct Gains {
        vector_3t orientation_kp;
        vector_3t orientation_kd;
        scalar_t leg_kp;
        scalar_t leg_kd;
    } gains;

    scalar_t springStiffness;

    Hopper();

    static matrix_3t cross(vector_3t q);
    static matrix_3t quat2Rot(quat_t q);

    void updateState(vector_t state);

    /*! @brief compute the torque (u_ff + u_feedback)
     * @param [in] quat_d_ desried quaternion
     * @param [in] omega_d desried angular rate
     * @param [in] length_des desried spring compression length
     * @param [in] u_des feed forward torque
     */
    void computeTorque(quat_t quat_d_, vector_3t omega_d, scalar_t length_des, vector_t u_des);

    /*! @brief  evaluate the forward dynamics
    *  @param [in] q  pos to evaluate the dynamics at
    *  @param [in] v  vel to evaluate the dynamics at
    *  @param [in] a  acc to evaluate the dynamics at
    *  @param [out] x_dot  the dynamics (dq, ddq)
    */
    vector_t f(const vector_t& q, const vector_t& v, const vector_t& a, const domain& d);

    /*! @brief  compute the linearizations of f
    *  @param [in]     q, v, a  - state to compute the jacobians at
    *  @param [out]    A, B, C  - df/dx, df/du, and the residual
    */
    void Df(const vector_t q, const vector_t v, const vector_t a, const domain d,
            matrix_t &A, matrix_t &B, matrix_t &C, const vector_t q0);

    /*! @brief  compute the discretization of the system
    *  @param [in]     Ac Continuous A matrix
    *  @param [in]     Bc Continuous B matrix
    *  @param [in]     Cc Continuous C matrix
    *  @param [out]     &Ad Discrete A matrix
    *  @param [out]     &Bd Discrete B matrix
    *  @param [out]     &Cd Discrete C matrix
    */
    void css2dss(const matrix_t &Ac, const matrix_t &Bc, const matrix_t &Cc,
            const float dt, matrix_t &Ad, matrix_t &Bd, matrix_t &Cd);

    /*!  @brief  compute the discretization of the system
    *  @param [in]     Ac, Bc, Cc - continuous time dynamics of the system
    *  @param [out]    Ad, Bd, Cd - discrete time dynamics of the system
    */
    void DiscreteDynamics(const vector_t &x, const vector_t &u, const domain &d, const float dt,
                          matrix_t &Ac, matrix_t &Bc, matrix_t &Cc,
                          matrix_t &Ad, matrix_t &Bd, matrix_t &Cd,
			  const vector_t q0);

    /*! @brief  compute the discrete dynamics at impact. calls impulse-dynamics
    *  @param [in]     q, v, J - state to compute the impact map at, and Jacobian of constraint
    *  @param [out]    x_plus   - the post impact state (q+, dq+)
    */
    vector_t delta_f(const vector_t q, const vector_t v, const domain d);

    /*! @brief  compute the jacobian of the discrete dynamics at impact
    *  @param [in]     q, v, J - state to compute the impact map at, and Jacobian of constraint
    *  @param [out]    A, B, C   -  ddelta_f/dx, ddelta_f/du, and the residual
    */
    void Ddelta_f(const vector_t q, const vector_t v, const domain d,
                  matrix_t &A, matrix_t &B, matrix_t &C, const vector_t q0);

};

#endif //HOPPER_HOPPER_H
