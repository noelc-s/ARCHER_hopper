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

// for joystick inputs
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>

#include <manif/manif.h>

// #include "../inc/Policy.h"
#include "../inc/ZeroDynamicsPolicy.h"
#include "../inc/Hopper.h"
#include "../inc/Types.h"
#include "../inc/UserInput.h"


#define PORT 8080
#define MAXLINE 1000

using namespace Eigen;
using namespace Hopper_t;


const static IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

// paramters for sending and receiving data
int *server_fd = new int;
int *new_socket = new int;
int valread;
struct sockaddr_in *address = new sockaddr_in;
int opt_socket = 1;
int addrlen = sizeof(*address);
scalar_t TX_torques[13+2*5] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0};
scalar_t RX_state[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


// setting up a virtual socket to stream data to and fro
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

// initializing the parameters for MPC and creating an instance p for the parameters.
struct Parameters {
    scalar_t dt;
    scalar_t roll_offset;
    scalar_t pitch_offset;
} p;

void setupGains(const std::string filepath) {
    // Read gain yaml
    YAML::Node config = YAML::LoadFile(filepath);
    p.dt = config["LowLevel"]["dt"].as<scalar_t>();
    p.roll_offset = config["roll_offset"].as<scalar_t>();
    p.pitch_offset = config["pitch_offset"].as<scalar_t>();
}

// Driver code
int main() {
    // initialize the socket, create an object to read the mpc parameters and read the gains
    setupSocket();

    // Read yaml
    const std::string gainYamlPath = "../config/gains.yaml";
    setupGains(gainYamlPath);

    // initializing the total states for the Hopper
    vector_t state(20);

    // Set up Data logging
    bool fileWrite = true;
    std::string dataLog = "../data/data.csv";
    std::string predictionLog = "../data/prediction.csv";
    std::ofstream fileHandle;
    fileHandle.open(dataLog);
    fileHandle <<"t,contact,x,y,z,q_w,q_x,q_y,q_z,l,wheel_pos1,wheel_pos2,wheel_pos3,x_dot,y_dot,z_dot,w_1,w_2,w_3,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,u_spring,tau_1,tau_2,tau_3,command_1,command_2,command_3"<<std::endl;
   
    // for discrete setup, t_last would be -1 to keep so that the planning happens for step 0:N-1?
    scalar_t t_last = -1;
    scalar_t dt_elapsed;

    // User Input, can be either Keyboard or Joystick
    // condition variable and mutex are used for multi-threading. Ignore for now.
    std::condition_variable cv;
    std::mutex m;
    vector_3t command;
    vector_2t command_interp;
    vector_3t dist;
    vector_2t offsets;
    offsets << p.roll_offset, p.pitch_offset;
    // old -- remove if comms work
    // std::thread userInput(getUserInput, std::ref(command), std::ref(cv), std::ref(m));
    // std::thread userInput(getJoystickInput, std::ref(offsets), std::ref(command), std::ref(dist), std::ref(cv), std::ref(m));
    
    // initializing the Pinnochio Model
    const std::string NNYamlPath = "../config/NN_gains.yaml";
    NNHopper hopper = NNHopper("../../models/low_level_trained_model.onnx", gainYamlPath);
    // Hopper hopper = Hopper();

    // Instantiate a new policy.
    RaibertPolicy policy = RaibertPolicy(gainYamlPath);
    // ZeroDynamicsPolicy policy = ZeroDynamicsPolicy("../../models/trained_model.onnx", gainYamlPath);

    UserInput readUserInput;
    // std::thread getUserInput(&UserInput::getKeyboardInput, &readUserInput, std::ref(command), std::ref(cv), std::ref(m));
    std::thread getUserInput(&UserInput::getJoystickInput, &readUserInput, std::ref(offsets), std::ref(command), std::ref(dist), std::ref(cv), std::ref(m));

    quat_t quat_des = Quaternion<scalar_t>(1,0,0,0);
    vector_3t omega_des;
    vector_4t u_des;

    // for infinity, do
    for (;;) {
      read(*new_socket, &RX_state, sizeof(RX_state));    
  
      Map<vector_t> state(RX_state, 20);
      dt_elapsed = state(0) - t_last;
      
      hopper.updateState(state);

      scalar_t x_d = 0;
      scalar_t y_d = 0;

      quat_t currentQuaterion = Quaternion<scalar_t>(state(4), state(5), state(6), state(7));


      static scalar_t yaw_0 = 2*asin(state(7)); // z part approximates initial yaw
      quat_t initial_yaw(cos(yaw_0/2),0,0,sin(yaw_0/2));
      quat_t rollPitch = Policy::Euler2Quaternion(-offsets[0],-offsets[1], 0);

      policy.updateOffsets(offsets);
      quat_des = policy.DesiredQuaternion(state(1), state(2), command(0)+state(1), command(1)+state(2), 
          state(8), state(9), dist(0));

        // here joystick is absolute position and yaw
      quat_des = policy.DesiredQuaternion(state(1), state(2), command(0), command(1), state(8), state(9), dist(0));
      quat_des = quat_des * initial_yaw * rollPitch; // applies rollPitch in local frame before yaw inverting

      quat_des = quat_des * initial_yaw * rollPitch; // applies rollPitch in local frame before yaw inverting
      omega_des = policy.DesiredOmega();
      u_des = policy.DesiredInputs();

      hopper.computeTorque(quat_des, omega_des, 0.1, u_des);
      t_last = state(0);

      vector_3t error;

      quat_t e = quat_des.inverse() * hopper.quat;
      manif::SO3Tangent<scalar_t> xi;
      auto e_ = manif::SO3<scalar_t>(e);
      xi = e_.log();
      error << xi.coeffs();

      // Log data
      if (fileWrite)
        fileHandle <<state[0] << "," << hopper.contact << "," << hopper.pos.transpose().format(CSVFormat)
          << "," << hopper.quat.coeffs().transpose().format(CSVFormat)
          << "," << quat_des.coeffs().transpose().format(CSVFormat)
          << "," << hopper.omega.transpose().format(CSVFormat)
          << "," << hopper.torque.transpose().format(CSVFormat)
          << "," << error.transpose().format(CSVFormat) 
          << "," << hopper.wheel_vel.transpose().format(CSVFormat)<< std::endl;

      for (int i = 0; i < 4; i++) {
          TX_torques[i] = hopper.torque[i];
      }

      // red dot
      TX_torques[11] = command(0)+state(1);
      TX_torques[12] = command(1)+state(2);

      // floating ghost body
      TX_torques[4] = state(1);
      TX_torques[5] = state(2);
      TX_torques[6] = 1;
      TX_torques[7] = quat_des.w();
      TX_torques[8] = quat_des.x();
      TX_torques[9] = quat_des.y();
      TX_torques[10] = quat_des.z();

      send(*new_socket, &TX_torques, sizeof(TX_torques), 0);
    }
}
