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

#include "../inc/Policy.h"
#include "../inc/Hopper.h"
#include "../inc/Types.h"
#include "../inc/utils.h"
#include "../inc/UserInput.h"
#include "../inc/rom.h"


using namespace Eigen;
using namespace Hopper_t;

const std::string gainYamlPath = "../config/gains.yaml";

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

quat_t quat_des = Quaternion<scalar_t>(1, 0, 0, 0);
vector_3t omega_des;
vector_t u_des(4);
vector_2t command_interp;
vector_t x_term(21);
vector_3t error;
manif::SO3Tangent<scalar_t> xi;
quat_t e;
quat_t quat_term;
quat_t IMU_quat;
vector_3t pos_term;
vector_2t offsets;
scalar_t reset;

UserInput readUserInput;

std::chrono::high_resolution_clock::time_point t1;
std::chrono::high_resolution_clock::time_point t2;

MPC_Params mpc_p;
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
std::atomic<bool> running(true);

matrix_t desired_command;
