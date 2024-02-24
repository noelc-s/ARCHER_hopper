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

#include "../inc/Policy.h"
#include "../inc/ZeroDynamicsPolicy.h"
#include "../inc/Hopper.h"
#include "../inc/Types.h"

#include "pinocchio/algorithm/jacobian.hpp"
//#include "pinocchio/algorithm/kinematics.hpp"

#define PORT 8080
#define MAXLINE 1000

using namespace Eigen;
using namespace Hopper_t;
using namespace pinocchio;


const static IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

scalar_t roll_increment = 0.01;
scalar_t pitch_increment = 0.01;

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

// Reads a joystick event from the joystick device.
// Returns 0 on success. Otherwise -1 is returned.
int read_event(int dev, struct js_event *event)
{
    ssize_t bytes;
    bytes = read(dev, event, sizeof(*event)); // read bytes sent by controller

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

// Current state of an axis.
struct axis_state {
    short x, y;
};

// simple list for button map
char buttons[4] = {'X','O','T','S'}; // cross, cricle, triangle, square

// get PS4 LS and RS joystick axis information
size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
  /* hard code for PS4 controller
     Left Stick:  +X is Axis 0 and right, +Y is Axis 1 and down
     Right Stick: +X is Axis 3 and right, +Y is Axis 4 and down 
  */
  size_t axis;

  // Left Stick (LS)
  if (event->number==0 || event->number==1) {
    axis = 0;  // arbitrarily call LS Axis 0
    if (event->number == 0)
      axes[axis].x = event->value;
    else
      axes[axis].y = event->value;
  }

  // Right Stick (RS)
  else {
    axis = 1;  // arbitrarily call RS Axis 1
    if (event->number == 3)
      axes[axis].x = event->value;
    else 
      axes[axis].y = event->value;
  }

  return axis;
}

void getJoystickInput(vector_2t &offsets, vector_3t &command, vector_2t &dist, std::condition_variable & cv, std::mutex & m)
{
  vector_3t input; input.setZero();
  std::chrono::seconds timeout(50000);
  const char *device;
  int js;
  struct js_event event;
  struct axis_state axes[3] = {0};
  size_t axis;
  dist.setZero();

  // if only one joystick input, almost always "/dev/input/js0"
  device = "/dev/input/js0";

  // joystick device index
  js = open(device, O_RDONLY); 
  if (js == -1)
      perror("Could not open joystick");
  else
      std::cout << "joystick connected" << std::endl;

  //scaling factor (joysticks vals in [-32767 , +32767], signed 16-bit)
  double comm_scale = 100000.;
  double dist_scale = 10000.;

  /* This loop will exit if the controller is unplugged. */
  while (read_event(js, &event) == 0)
  {
    switch(event.type) {
      
      // moving a joystick
      case JS_EVENT_AXIS:
        axis = get_axis_state(&event, axes);
        if (axis == 0) { 
          command << axes[axis].x / comm_scale, -axes[axis].y / comm_scale, 0; // Left Joy Stick
        //   std::cout << "Command: " << command[0] << ", " << command[1] << std::endl;
        }
        if (axis == 1) {
          dist << axes[axis].x / dist_scale, -axes[axis].y / dist_scale; // Right Joy Stick
        //   std::cout << "Disturbance: " << dist[0] << ", " << dist[1] << std::endl;
        }
        break;

      // pressed a button
      case JS_EVENT_BUTTON:
        if (event.number == 5 && event.value == 1)
          std::cout << "reset" << std::endl;
        // can do something cool with buttons
        if (event.number == 0 && event.value == 1)
          offsets[1] -= pitch_increment;
        if (event.number == 1 && event.value == 1)
          offsets[0] += roll_increment;
        if (event.number == 2 && event.value == 1)
          offsets[1] += pitch_increment;
        if (event.number == 3 && event.value == 1)
          offsets[0] -= roll_increment;
        std::cout << "Offsets: " << offsets.transpose() << std::endl;
        break;
      
      // ignore init events
      default:
        break;
    }
  }

  close(js);
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
    const std::string yamlPath = "../config/gains.yaml";
    setupGains(yamlPath);

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
    fileHandle <<"t,contact,x,y,z,q_w,q_x,q_y,q_z,l,wheel_pos1,wheel_pos2,wheel_pos3,x_dot,y_dot,z_dot,w_1,w_2,w_3,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,u_spring,tau_1,tau_2,tau_3,command_1,command_2,command_3"<<std::endl;
    // fileHandle << "t,x,y,z,q_w,q_x,q_y,q_z,x_dot,y_dot,z_dot,w_1,w_2,w_3,contact,l,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,z_acc";
    // std::ofstream fileHandleDebug;
    // fileHandleDebug.open(predictionLog);
    // fileHandleDebug << "t,x,y,z,q_w,q_x,q_y,q_z,x_dot,y_dot,z_dot,w_1,w_2,w_3,contact,l,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,z_acc";

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
    vector_2t dist;
    vector_2t offsets;
    offsets.setZero();
    // std::thread userInput(getUserInput, std::ref(command), std::ref(cv), std::ref(m));
    std::thread userInput(getJoystickInput, std::ref(offsets), std::ref(command), std::ref(dist), std::ref(cv), std::ref(m));

    quat_t quat_des = Quaternion<scalar_t>(1,0,0,0);
    vector_3t omega_des;
    vector_4t u_des;

    // Instantiate a new policy.
    RaibertPolicy policy = RaibertPolicy(yamlPath);
    // ZeroDynamicsPolicy policy = ZeroDynamicsPolicy("../../models/trained_model.onnx", yamlPath);

    // MH
    // for infinity, do
    for (;;) {
      read(*new_socket, &RX_state, sizeof(RX_state));
  
      Map<vector_t> state(RX_state, 20);
      dt_elapsed = state(0) - t_last;
      
      hopper.updateState(state);

      scalar_t x_d = 0;
      scalar_t y_d = 0;

      quat_t currentQuaterion = Quaternion<scalar_t>(state(4), state(5), state(6), state(7));
      vector_3t currentEulerAngles = currentQuaterion.toRotationMatrix().eulerAngles(0, 1, 2);
    
      policy.updateOffsets(offsets);
      quat_des = policy.DesiredQuaternion(state(1), state(2), command(0)+state(1), command(1)+state(2), 
          state(8), state(9), dist(0));
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
