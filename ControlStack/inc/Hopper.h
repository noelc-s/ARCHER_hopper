#pragma once

#include <iostream>
#include "Types.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include "yaml-cpp/yaml.h"
#include <numeric>
#include "pinocchio_wrapper.h"

using namespace Hopper_t;

template <typename T>
T vectorProduct(const std::vector<T>& v)
    { return std::accumulate(v.begin(), v.end(), 1, std::multiplies<T>()); }

class Hopper {

public:

    struct State {
        scalar_t t;
        vector_3t pos;
        quat_t quat;
        vector_3t vel;
        vector_3t omega;
        scalar_t contact;
        scalar_t last_impact_time;
        scalar_t last_flight_time;
        scalar_t leg_pos;
        scalar_t leg_vel;
        vector_3t wheel_vel;
        vector_t q;
        vector_t v;
        domain dom;
    } state_;

    vector_4t torque;

    quat_t quat_actuator;

    struct Gains {
        vector_3t orientation_kp;
        vector_3t orientation_kd;
        scalar_t leg_kp;
        scalar_t leg_kd;
    } gains;

    std::unique_ptr<PinocchioWrapper> pinocchioWrapper;

    scalar_t springStiffness;

    Hopper(const std::string yamlFile);

    void updateState(vector_t state);

    /*!  @brief  compute the discretization of the system
    *  @param [in]     Ac, Bc, Cc - continuous time dynamics of the system
    *  @param [out]    Ad, Bd, Cd - discrete time dynamics of the system
    */
    void DiscreteDynamics(const vector_t &x, const vector_t &u, const domain &d, const float dt,
                          matrix_t &Ac, matrix_t &Bc, matrix_t &Cc,
                          matrix_t &Ad, matrix_t &Bd, matrix_t &Cd,
			              const vector_t q0);


    /*! @brief compute the torque (u_ff + u_feedback)
     * @param [in] quat_d_ desried quaternion
     * @param [in] omega_d desried angular rate
     * @param [in] length_des desried spring compression length
     * @param [in] u_des feed forward torque
     */
    void computeTorque(quat_t quat_d_, vector_3t omega_d, scalar_t length_des, vector_t u_des);

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

};
