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
#include<netinet/in.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include <manif/manif.h>

#include "../inc/Hopper.h"
#include "../inc/Types.h"
#include "../inc/MPC.h"

#include "pinocchio/algorithm/jacobian.hpp"
//#include "pinocchio/algorithm/kinematics.hpp"

#define PORT 8080
#define MAXLINE 1000

using namespace Eigen;
using namespace Hopper_t;
using namespace pinocchio;


const static IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

// MH
// reads the user input from the command line and saves it to the 3x1 vector input
// what are the inputs?
static vector_3t getInput() {
  
  vector_3t input;
  std::string line;
  getline(std::cin, line);
  
  std::istringstream iss(line);
  int pos = 0;
  scalar_t num;
  
  while(iss >> num) {
    input[pos] = num; pos++;
  }
  
  return input;
}

// https://stackoverflow.com/questions/41505451/c-multi-threading-communication-between-threads
// https://stackoverflow.com/questions/6171132/non-blocking-console-input-c
// MH
// waits for the user input and asyncronously wait for the input, if everything is okay, it reads the input to the command
// don't read too much into it
void getUserInput(vector_3t &command, std::condition_variable & cv, std::mutex & m)
{
  vector_3t input; input.setZero();
  
  std::chrono::seconds timeout(50000);
  
  while(1) {
   std::future<vector_3t> future = std::async(getInput);
   
   if (future.wait_for(timeout) == std::future_status::ready)
        input = future.get();
   
   command << input;
  }
}
// MH
// what are TX_torques and RX_state? The name suggests that it is transmitting torque commands and recieving the system state. 
// The number of states is consistent, what is the number of torques?

    int *server_fd = new int;
    int *new_socket = new int;
    int valread;
    struct sockaddr_in *address = new sockaddr_in;
    int opt_socket = 1;
    int addrlen = sizeof(*address);
    scalar_t TX_torques[13+2*5] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0};
    scalar_t RX_state[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// MH
// sets up a socket between something and something. It is reading from the port 8080, so something needs to publish on that port.
// maybe states from mujoco?
void setupSocket() {
// Socket stuff
    if ((*server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(*server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                   &opt_socket, sizeof(opt_socket))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address->sin_family = AF_INET;
    address->sin_addr.s_addr = INADDR_ANY;
    address->sin_port = htons(PORT);

    // Forcefully attaching socket to the port 8080
    if (bind(*server_fd, (struct sockaddr *) address,
             sizeof(*address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(*server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((*new_socket = accept(*server_fd, (struct sockaddr *) address,
                              (socklen_t *) &addrlen)) < 0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }
}

// MH 
// initializing the parameters for MPC and creating an instance p for the parameters.
// These parameters can the be evaluated by defining the instance with the same name, for example
// Parameters p;
// p.dt = 1212;
struct Parameters {
    scalar_t dt;
} p;

// MH
// read the yaml file and set the values for the objects p from the Structure parameters and mpc from the structure MPC_Params

void setupGains(const std::string filepath) {
    // Read gain yaml
    YAML::Node config = YAML::LoadFile(filepath);
    p.dt = config["LowLevel"]["dt"].as<scalar_t>();
}

// Driver code
int main() {
// MH
// initialize the socket, create an object to read the mpc parameters and read the gains
    setupSocket();    
    setupGains("../config/gains.yaml");

    // Read yaml

    // MH
    // initializing the total states for the Hopper
    vector_t state(20);

    // MH
    // initializing the Pinnochio Model
    Hopper hopper = Hopper();


    // Set up Data logging
    bool fileWrite = true;
    std::string dataLog = "../data/data.csv";
    std::string predictionLog = "../data/prediction.csv";
    std::ofstream fileHandle;
    fileHandle.open(dataLog);
    fileHandle << "t,x,y,z,q_w,q_x,q_y,q_z,x_dot,y_dot,z_dot,w_1,w_2,w_3,contact,l,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,z_acc";
    std::ofstream fileHandleDebug;
    fileHandleDebug.open(predictionLog);
    fileHandleDebug << "t,x,y,z,q_w,q_x,q_y,q_z,x_dot,y_dot,z_dot,w_1,w_2,w_3,contact,l,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,z_acc";

    // MH
    // for discrete setup, t_last would be -1 to keep so that the planning happens for step 0:N-1?
    scalar_t t_last = -1;
    scalar_t dt_elapsed;


    // MH
    // condition variable and mutex are used for multi-threading. Ignore for now.
    // assuming, command is the global position of the robot? command_interp is the function for performing a particular action?
    std::condition_variable cv;
    std::mutex m;
    vector_3t command;
    vector_2t command_interp;
    std::thread userInput(getUserInput, std::ref(command), std::ref(cv), std::ref(m));

    // MH
    // for infinity, do
    for (;;) {
        read(*new_socket, &RX_state, sizeof(RX_state));
   
        Map<vector_t> state(RX_state, 20);
        dt_elapsed = state(0) - t_last;
        
        hopper.updateState(state);
       
        quat_des = Quaternion<scalar_t>(1,0,0,0);
        vector_3t omega_des;
        vector_4t u_des;

        if (dt_elapsed > p.dt) {
            // Uncomment below to isolate the low level controller for testing
            // quat_des = Quaternion<scalar_t>(1,0,0,0);
            // omega_des << 0,0,0;
            // u_des << 0,0,0,0;

            quat_des = DesiredQuaternion();
            omega_des = DesiredOmega();
            u_des = DesiredInputs();

            hopper.computeTorque(quat_des, omega_des, 0.1, u_des);
            t_last = state(0);
        }

            for (int i = 0; i < 4; i++) {
                TX_torques[i] = hopper.torque[i];
            }


            send(*new_socket, &TX_torques, sizeof(TX_torques), 0);
        }

}
