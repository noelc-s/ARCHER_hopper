#pragma once

#include "../inc/Types.h"
#include "../inc/UserInput.h"
#include <mutex>
#include <condition_variable>
#include <thread>
#include <future>
#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include <manif/manif.h>

// #include "../inc/MPC.h"

#define PORT 8080

using namespace Eigen;
using namespace Hopper_t;

const static IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

static vector_3t getInput();

void hi(int num);

void setupSocket(int &server_fd, int &new_socket, struct sockaddr_in &address, int opt_socket, int &addrlen);

void setupGains(const std::string filepath, MPC_Parameters &mpc_p, Parameters &p); // MPC::MPC_Parameters &mpc_p, 

vector_3t Quaternion2Euler(const quat_t &q);

quat_t minus(quat_t q_1, quat_t q_2);
quat_t plus(quat_t q_1, quat_t q_2);
scalar_t extract_yaw(quat_t q);

/*! @brief  evaluate the forward dynamics
*  @param [in] roll  roll angle of the body frame wrt the world frame
*  @param [in] pitch  pitch angle of the body frame wrt the world frame
*  @param [in] yaw  yaw angle of the body frame wrt the world frame
*  @param [out] quaternion  quaternion representation of the orientation
*/
inline quat_t Euler2Quaternion(scalar_t roll, scalar_t pitch, scalar_t yaw) {
    return AngleAxisd(roll, Vector3d::UnitX())
                * AngleAxisd(pitch, Vector3d::UnitY())
                * AngleAxisd(yaw, Vector3d::UnitZ());
}

/*! @brief Take the state and apply the log of the orientation to get elements of the Lie Algebra
* @param[in] x Lie Group elements
* @param[out] xi Lie Algebra elements
*/
vector_t Log(vector_t x);

/*! @brief Take elements of the Lie Algebra and Exp them to the Lie Group
* @param[in] xi Lie Algebra elements
* @param[out] x Lie Group elements
*/
vector_t Exp(vector_t xi);

/*! @brief apply q0_inverse and then perform Log
* @param[in] qk the Lie Group element
* @param[in] q0 the base point
* @param[out] xik the Lie Algebra element
*/
vector_t qk_to_xik(vector_t qk, vector_t q0);

/*! @brief apply q0 and then perform Exp
*
* @param[in] xik the Lie Algebra element
* @param[in] q0 the base point
* @param[out] qk the Lie Group element
*/
vector_t xik_to_qk(vector_t xik, vector_t q0);

/*! @brief Convert local frame to the global frame
*
* @param[in] x_l the local frame coordinate
* @param[out] x_g the global frame coordinate
*/
vector_t local2global(vector_t x_l);

/*! @brief Convert global frame to the local frame
*
* @param[in] x_g the global frame coordinate
* @param[out] x_l the local frame coordinate
*/
vector_t global2local(vector_t x_g);

matrix_3t cross(vector_3t q);
matrix_3t quat2Rot(quat_t q);