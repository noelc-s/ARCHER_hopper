# pragma once
//
// for joystick inputs
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <Python.h>

// server program for udp connection
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <strings.h>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <future>
#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include <manif/manif.h>

#include "../inc/Hopper.h"
#include "../inc/Types.h"
#include "../inc/MPC.h"
#include "../inc/Graph.h"
#include "pinocchio/algorithm/jacobian.hpp"

#include <cassert>

//#include "pinocchio/algorithm/kinematics.hpp"
//#define port 8080
#define MAXLINE 1000
using namespace Eigen;
using namespace Hopper_t;
using namespace pinocchio;

enum ProgramState {
  RUNNING = 1,
  STOPPED = 0,
  RESET = -1,
  KILL = -10
};

struct Parameters {
    scalar_t dt;
    scalar_t MPC_dt_flight;
    scalar_t MPC_dt_ground;
    scalar_t MPC_dt_replan;
    int stop_index; 
} p;

class C {
public:
    virtual ~C() { }
    virtual void run() = 0;
};

void call_run(C *c) {
    c->run();
}

class Controller : public C {
  public:
    Controller();
    // flag to pass to simulator
    // 1 = continue simulation as normal
    // 0 = stop simulation
    // -1 = reset simulation
    ProgramState programState_ = RUNNING;
    scalar_t t_last = -1;
    scalar_t t_last_MPC = -1;
    scalar_t dt_elapsed;
    scalar_t dt_elapsed_MPC;

    scalar_t TX_torques[13+2*5+2+1] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    vector_t initialCondition_;  // pos[3], rpy[3], v[3], omega[3] 
    vector_t goalState_;         // pos[3], rpy[3], v[3], omega[3] 
    scalar_t objVal;             // MPC primal objective value
    scalar_t num_hops;                // number of hops

    vector_array_t stateSequence_; // sequence of goal states
    scalar_array_t paramsSequence_;      // parameterization of goal states

    void setInitialState(vector_t initialCondition);
    void setGoalState(vector_t goalState);
    void setStateSequence(vector_array_t stateSequence, scalar_array_t paramsSequence);
    
    void resetSimulation(vector_t x0);
    void stopSimulation();
    void killSimulation();
    void startSimulation();

    void run() override;
    scalar_t duration = -1;

    std::unique_ptr<uint16_t> port;
 
    // State variables
    vector_t x;
    vector_t q;
    vector_t v;
    vector_t q_local;
    vector_t v_local;
    vector_t q_global;
    vector_t v_global;
    vector_t tau;

    void getStateUpdate(Hopper hopper);
};

const static IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");
scalar_t RX_state[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static vector_3t getInput();
void getUserInput(vector_3t &command, std::condition_variable & cv, std::mutex & m);
int read_event(int dev, struct js_event *event);
size_t get_axis_state(struct js_event *event, struct axis_state axes[3]);
void getJoystickInput(vector_3t &command, vector_2t &dist, std::condition_variable & cv, std::mutex & m);
void setupGains(const std::string filepath, MPC::MPC_Params &mpc_p);
void setupSocket(int* new_socket, int* server_fd, struct sockaddr_in* address, uint32_t PORT);
vector_t rpy_to_quat(vector_t rpy);

vector_t get_configIC();
void setupSocket_sendIC(vector_t initialCondition);

// Current state of an axis.
struct axis_state {
    short x, y;
};
// simple list for button map
char buttons[4] = {'X','O','T','S'}; // cross, cricle, triangle, square



