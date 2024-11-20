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

// for joystick inputs
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>

#include <manif/manif.h>
#include <signal.h>

#include "../inc/Policy.h"
#include "../inc/Hopper.h"
#include "../inc/Types.h"
// #include "../inc/MPC.h"
#include "../inc/utils.h"
#include "../inc/UserInput.h"
#include "../src/utils_hardware.cpp"
#include "../inc/rom.h"

using namespace Eigen;
using namespace Hopper_t;


quat_t quat_a, quat_d;
vector_3t omega_a, omega_d, torque;
scalar_t Q_w, Q_x, Q_y, Q_z;
scalar_t w_x, w_y, w_z;
//torques to currents, assumes torques is an array of floats
scalar_t const_wheels = 0.083;
bool init = false;
vector_t state_init(3);

bool fileWrite = true;
std::string dataLog = "../data/data_hardware.csv";
std::ofstream fileHandle;

// dist?
std::condition_variable cv;
std::mutex m;
std::atomic<bool> running(true);

std::chrono::high_resolution_clock::time_point t_loop;
std::chrono::high_resolution_clock::time_point t_policy;
std::chrono::high_resolution_clock::time_point t_lowlevel;
std::chrono::high_resolution_clock::time_point tstart;

const std::string gainYamlPath = "../config/gains_hardware.yaml";

vector_t state(20);
scalar_t t_last = -1;
scalar_t dt_elapsed;
quat_t quat_des;
vector_3t omega_des;
vector_t u_des(4);

vector_2t offsets;

UserInput readUserInput;

vector_3t last_state;

quat_t quat_optitrack;
matrix_t desired_command;

vector_3t error;
quat_t e;
manif::SO3Tangent<scalar_t> xi;

scalar_t reset;








// // 4 torques, 7 terminal s SE(3) state, 2 command, 2*5 solution horizon COM xy pos
// scalar_t TX_torques[4 + 7 + 2 + 2 * 5] = {};
//  // time, pos, quat, vel, omega, contact, leg_pos, leg_vel, wheel_vel
// scalar_t RX_state[20] = {};



// int ind;
// vector_t x_term(21);


// quat_t IMU_quat;


// // MPC::MPC_Parameters mpc_p;


// // Socket Stuff
// int server_fd;
// int new_socket;
// // int valread;
// struct sockaddr_in address;
// int opt_socket = 1;
// int addrlen;

// vector_3t command;
// vector_2t command_interp;

// matrix_t x_pred(21,2);
// matrix_t u_pred(4,1);

// quat_t quat_term;
// vector_3t pos_term;