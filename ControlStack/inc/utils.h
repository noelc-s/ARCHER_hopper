#ifndef UTILS_H
#define UTILS_H

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

// #include "../inc/MPC.h"

#define PORT 8080

using namespace Eigen;
using namespace Hopper_t;

const static IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

static vector_3t getInput();

void hi(int num);

struct Parameters {
    scalar_t dt;
    scalar_t MPC_dt_flight;
    scalar_t MPC_dt_ground;
    scalar_t MPC_dt_replan;
    scalar_t roll_offset;
    scalar_t pitch_offset;
    scalar_t yaw_drift;
    std::string model_name;
    scalar_t v_max;
    scalar_t a_max;
    scalar_t dt_replan;
    int horizon;
    std::string rom_type;
    int stop_index; 
    double o_r, o_x, o_y;
};

struct MPC_Params {
    int N;
	int SQP_iter;
    vector_t stateScaling;
    vector_t inputScaling;
    scalar_t discountFactor;
	scalar_t dt_flight;
	scalar_t dt_ground;
    scalar_t MPC_dt_replan;
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
};

void setupSocket(int &server_fd, int &new_socket, struct sockaddr_in &address, int opt_socket, int &addrlen);

void setupGains(const std::string filepath, MPC_Params &mpc_p, Parameters &p); // MPC::MPC_Params &mpc_p, 

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
static quat_t Euler2Quaternion(scalar_t roll, scalar_t pitch, scalar_t yaw) {
    return AngleAxisd(roll, Vector3d::UnitX())
                * AngleAxisd(pitch, Vector3d::UnitY())
                * AngleAxisd(yaw, Vector3d::UnitZ());
}

#endif