// server program for udp connection
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <strings.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include <manif/manif.h>

#include "../inc/Hopper.h"
#include "../inc/Types.h"
#include "../inc/MPC.h"
#include "../inc/utils.h"

using namespace Eigen;
using namespace Hopper_t;
using namespace pinocchio;

// 4 torques, 7 terminal s SE(3) state, 2 command, 2*5 solution horizon COM xy pos
scalar_t TX_torques[4 + 7 + 2 + 2 * 5] = {};
 // time, pos, quat, vel, omega, contact, leg_pos, leg_vel, wheel_vel
scalar_t RX_state[20] = {};

bool fileWrite = true;
std::string dataLog = "../data/data.csv";
std::string predictionLog = "../data/prediction.csv";
std::ofstream fileHandle;
std::ofstream fileHandleDebug;

int ind;
scalar_t t_last = -1;
scalar_t dt_elapsed;
scalar_t t_last_MPC = -1;
scalar_t dt_elapsed_MPC;

quat_t quat_des = Quaternion<scalar_t>(1, 0, 0, 0);
vector_3t omega_des;
vector_t u_des(4);
vector_3t command;
vector_2t command_interp;
matrix_t x_pred(21, 2);
matrix_t u_pred(4, 1);
vector_t x_term(21);
vector_t q0(21);
vector_t q0_local(21);
vector_t v_global(6);
vector_t v_local(6);
vector_t x_global(21);
vector_t x_local(21);
vector_t xi_local(21);
quat_t quat_term;
vector_3t pos_term;

std::chrono::high_resolution_clock::time_point t1;
std::chrono::high_resolution_clock::time_point t2;

MPC::MPC_Params mpc_p;
Parameters p;

// Socket Stuff
int server_fd;
int new_socket;
// int valread;
struct sockaddr_in address;
int opt_socket = 1;
int addrlen;

// Threading stuff
std::condition_variable cv;
std::mutex m;