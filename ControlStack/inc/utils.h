#ifndef UTILS_H
#define UTILS_H

#include "../inc/Types.h"
#include <mutex>
#include <condition_variable>
#include <thread>
#include <future>
#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>

#include "../inc/MPC.h"

#define PORT 8080

using namespace Eigen;
using namespace Hopper_t;

const static IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

static vector_3t getInput();

void getUserInput(vector_3t &command, std::condition_variable & cv, std::mutex & m);

int *server_fd;
int *new_socket;
int valread;
struct sockaddr_in *address;
int opt_socket;
int addrlen;
scalar_t TX_torques[13+2*5];
scalar_t RX_state[20];

void setupSocket();

struct Parameters {
    scalar_t dt;
    scalar_t MPC_dt_flight;
    scalar_t MPC_dt_ground;
    scalar_t MPC_dt_replan;
    int stop_index; 
} p;

void setupGains(const std::string filepath, MPC::MPC_Params &mpc_p);

#endif