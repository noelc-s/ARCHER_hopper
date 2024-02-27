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
#include <signal.h>
#include<thread>
#include <unistd.h> // close socket

// for joystick inputs
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/String.h"

#include <manif/manif.h>

#include "../inc/UserInput.h"
#include "../inc/Hopper.h"
#include "../inc/Types.h"
#include "../inc/Policy.h"
#include "../inc/ZeroDynamicsPolicy.h"

#define MAXLINE 1000
#define MAX 48
#define STATE_SIZE 40
#define MSG_SIZE 48+4
#define PORT 8888
#define SA struct sockaddr

using namespace Eigen;
using namespace Hopper_t;


const static IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

int sock;
char buff[52];
char send_buff[40];
float states[13];
vector_t ESPstate(13);
float desstate[10];
std::mutex state_mtx;
std::mutex des_state_mtx;
sockaddr_in source, destination;
sockaddr_in senderAddr;
socklen_t destinationAddrLen;
socklen_t senderAddrLen;

bool ethernet_send_reset = false;

struct Parameters {
    std::vector<scalar_t> orientation_kp;
    std::vector<scalar_t> orientation_kd;
    scalar_t leg_kp;
    scalar_t leg_kd;
    scalar_t dt;
    scalar_t frameOffset;
    scalar_t markerOffset;
    int predHorizon;
    int stop_index; 
    vector_t gains;
    std::vector<scalar_t> p0;
    scalar_t roll_offset;
    scalar_t pitch_offset;
} p;

void signal_callback_handler(int signum) {
        std::cout << "Caught signal " << signum << std::endl;
   // Terminate program
   exit(signum);
}

struct State {
  scalar_t x;
  scalar_t y;
  scalar_t z;
  scalar_t x_dot;
  scalar_t y_dot;
  scalar_t z_dot;
  scalar_t q_w;
  scalar_t q_x;
  scalar_t q_y;
  scalar_t q_z;
  scalar_t w_x;
  scalar_t w_y;
  scalar_t w_z;
} OptiState;

void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  OptiState.x = msg->pose.position.x;
  OptiState.y = msg->pose.position.y;
  OptiState.z = msg->pose.position.z + p.frameOffset + p.markerOffset;
  OptiState.q_w = msg->pose.orientation.w;
  OptiState.q_x = msg->pose.orientation.x;
  OptiState.q_y = msg->pose.orientation.y;
  OptiState.q_z = msg->pose.orientation.z;
}

void setupGains(const std::string filepath) {
    YAML::Node config = YAML::LoadFile(filepath);
    p.orientation_kp = config["Orientation"]["Kp"].as<std::vector<scalar_t>>();
    p.orientation_kd = config["Orientation"]["Kd"].as<std::vector<scalar_t>>();
    p.leg_kp = config["Leg"]["Kp"].as<scalar_t>();
    p.leg_kd = config["Leg"]["Kd"].as<scalar_t>();
    p.dt = config["Debug"]["dt"].as<scalar_t>();
    p.predHorizon = config["Debug"]["predHorizon"].as<int>();
    p.stop_index = config["Debug"]["stopIndex"].as<int>();
    p.p0 = config["Simulator"]["p0"].as<std::vector<scalar_t>>();
    p.roll_offset = config["roll_offset"].as<scalar_t>();
    p.pitch_offset = config["pitch_offset"].as<scalar_t>();
    p.gains.resize(8);
    p.gains << p.orientation_kp[0], p.orientation_kp[1], p.orientation_kp[2],
            p.orientation_kd[0], p.orientation_kd[1], p.orientation_kd[2],
            p.leg_kp, p.leg_kd;
    std::cout<<"setupGains OK"<<std::endl;
}

volatile bool ESP_initialized = false;

void getStateFromEthernet() {

  ssize_t recvBytes = 0;
  ssize_t n_bytes = 0;

  while(1) {

    // encode send_buff
  {std::lock_guard<std::mutex> lck(des_state_mtx);
  memcpy(send_buff, desstate, 10*4);
  }
  
  // std::cout<<"Writing..."<<std::endl;
  if (ethernet_send_reset == true) {
    send_buff[0] = 1; send_buff[1] = 2; send_buff[2] = 3; send_buff[3] = 4;
    send_buff[4] = 5; send_buff[5] = 6; send_buff[6] = 7; send_buff[7] = 8;
    ethernet_send_reset = false;
  }
  
  n_bytes = ::sendto(sock, send_buff, sizeof(send_buff), 0, reinterpret_cast<sockaddr*>(&destination), destinationAddrLen);
  
  //receive string states, ESP8266 -> PC
  // std::cout<<"Reading..."<<std::endl;
  recvBytes = ::recvfrom(sock, buff, sizeof(buff), 0, reinterpret_cast<sockaddr*>(&senderAddr), &senderAddrLen);

  float data[13] = {0.0,0.0,0.0,0.0,0.0, 0.0, 0.0,
                            0.0,0.0,0.0,0.0,0.0,0.0};
  std::memcpy(data, buff, sizeof(data));

  memcpy(&states, buff, 13*sizeof(float));
  quat_t quat_a = quat_t(states[6], states[7], states[8], states[9]);
  quat_a.normalize();
  {std::lock_guard<std::mutex> lck(state_mtx);
  ESPstate << states[0], states[1], states[2], states[3], states[4], states[5], quat_a.w(), quat_a.x(), quat_a.y(), quat_a.z(), states[10], states[11], states[12];
  ESP_initialized = true;
  }

  }

  ::close(sock); 
}

void setupSocket() {

    std::string hostname{"10.0.0.6"};               // IP Adress of the MACHINE communicating with the teensy
    std::string destname{"10.0.0.7"};               // IP Address of the TEENSY communicating with the machine
    uint16_t port = 4333;                           // Port over which you want to communicate

    sock = ::socket(AF_INET, SOCK_DGRAM, 0);    // AF_INET for IPV4, SOCK_DGRAM for UDP, 0 for IP (protocol value)

    if (sock == -1) {
      printf("socket creation failed...\n");
      exit(0);
    }
    else{
      printf("Socket successfully created..\n");
    }

    source.sin_family = AF_INET;
    source.sin_port = htons(port);
    source.sin_addr.s_addr = inet_addr(hostname.c_str());

    destination.sin_family = AF_INET;
    destination.sin_port = htons(port);
    destination.sin_addr.s_addr = inet_addr(destname.c_str());
    destinationAddrLen = sizeof(destination);

    int enabled = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled));

    int val = bind(sock, (struct sockaddr*)&source,
             sizeof(source));

    senderAddr.sin_family = AF_INET;
    senderAddr.sin_port = htons(port);
    senderAddr.sin_addr.s_addr = inet_addr(hostname.c_str());
    senderAddrLen = sizeof(senderAddr);
    sleep(1);
}

// Driver code
int main(int argc, char **argv){

    quat_t quat_a, quat_d;
    vector_3t omega_a, omega_d, torque;
    scalar_t Q_w, Q_x, Q_y, Q_z;
    scalar_t w_x, w_y, w_z;
    //torques to currents, assumes torques is an array of floats
    scalar_t const_wheels = 0.083;
    bool init = false;
    vector_t state_init(3);
    ESPstate.setZero();

    bool fileWrite = true;
    std::string dataLog = "../data/data_hardware.csv";
    std::ofstream fileHandle;
    fileHandle.open(dataLog);
    fileHandle <<"t,contact,x,y,z,q_w,q_x,q_y,q_z,l,wheel_pos1,wheel_pos2,wheel_pos3,x_dot,y_dot,z_dot,w_1,w_2,w_3,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,u_spring,tau_1,tau_2,tau_3,command_1,command_2,command_3,quat_d_w, quat_d_x, quat_d_y, quat_d_z"<<std::endl;

    desstate[0] = 1;
    desstate[1] = 0;
    desstate[2] = 0;
    desstate[3] = 0;
    desstate[4] = 0;
    desstate[5] = 0;
    desstate[6] = 0;
    desstate[7] = 0;
    desstate[8] = 0;
    desstate[9] = 0;

    signal(SIGINT, signal_callback_handler);
    setupSocket();
    std::thread thread_object(getStateFromEthernet);

    std::chrono::high_resolution_clock::time_point t1;
    std::chrono::high_resolution_clock::time_point t2;
    std::chrono::high_resolution_clock::time_point tstart;

    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////

    // Read yaml
    std:: string yamlPath = "../config/gains_hardware.yaml";
    setupGains(yamlPath);

    vector_t state(20);

    Hopper hopper = Hopper();

    vector_t q(11);
    vector_t v(10);
    vector_t q_local(11);
    vector_t v_local(10);
    vector_t q_global(11);
    vector_t v_global(10);
    vector_t tau(10);
    // Pinocchio states: pos, quat, leg, flywheeels

    int index = 1;

    scalar_t t_last = -1;
    scalar_t dt_elapsed;

    quat_t quat_des = Quaternion<scalar_t>(1,0,0,0);
    vector_3t omega_des;
    omega_des.setZero();
    vector_t u_des(4);
    u_des.setZero();

    std::condition_variable cv;
    std::mutex m;
    vector_3t command;
    vector_2t command_interp;
    vector_2t dist;
    vector_2t offsets;
    offsets << p.roll_offset, p.pitch_offset;

    UserInput readUserInput;
    // std::thread getUserInput(&UserInput::getKeyboardInput, &readUserInput, std::ref(command), std::ref(cv), std::ref(m));
    std::thread getUserInput(&UserInput::getJoystickInput, &readUserInput, std::ref(offsets), std::ref(command), std::ref(dist), std::ref(cv), std::ref(m));

    vector_3t last_state;


    // ROS stuff
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    // if (argc<1) {
      ros::Subscriber sub = n.subscribe("/vrpn_client_node/hopper/pose", 200, chatterCallback);
    // } else {
    //   OptiState.q_w = 1;
    //   OptiState.q_x = 0;
    //   OptiState.q_y = 0;
    //   OptiState.q_z = 0;
    //   OptiState.w_x = 0;
    //   OptiState.w_y = 0;
    //   OptiState.w_z = 0;
    //   OptiState.x = 0;
    //   OptiState.y = 0;
    //   OptiState.z = 0;
    //   OptiState.x_dot = 0;
    //   OptiState.y_dot = 0;
    //   OptiState.z_dot = 0;
    // }
    

    while(!ESP_initialized) {};

    vector_3t current_vel, previous_vel;
    scalar_t dt = 1;
    std::chrono::high_resolution_clock::time_point last_t_state_log;

    tstart = std::chrono::high_resolution_clock::now();
    t2 = tstart;

    RaibertPolicy policy = RaibertPolicy(yamlPath);
    // ZeroDynamicsPolicy policy = ZeroDynamicsPolicy("../../models/trained_model.onnx", yamlPath);

    while(ros::ok()){
      ros::spinOnce();
      t1 = std::chrono::high_resolution_clock::now();

      if (!init) {
        state_init << OptiState.x,OptiState.y,OptiState.z;
	      last_state  << OptiState.x-state_init(0),OptiState.y-state_init(1),OptiState.z;
	      current_vel << 0,0,0;
	      previous_vel << 0,0,0;
          // dt = p.MPC_dt_replan;
          init = true;
      } else {
        dt = std::chrono::duration_cast<std::chrono::nanoseconds>(t1-last_t_state_log).count()*1e-9;
      }

      {std::lock_guard<std::mutex> lck(state_mtx);
      current_vel << ((OptiState.x-state_init(0))-last_state(0))/dt, ((OptiState.y-state_init(1))-last_state(1))/dt, ((OptiState.z)-last_state(2))/dt;
      state << std::chrono::duration_cast<std::chrono::milliseconds>(t1-tstart).count()*1e-3,
            OptiState.x-state_init(0),OptiState.y-state_init(1),OptiState.z,
            ESPstate(6),ESPstate(7),ESPstate(8),ESPstate(9),
            (current_vel(0)+previous_vel(0))/2, (current_vel(1)+previous_vel(1))/2, (current_vel(2)+previous_vel(2))/2,
            //current_vel(0), current_vel(1), current_vel(2), 
            ESPstate(3),ESPstate(4),ESPstate(5),
            ESPstate(10),ESPstate(11),ESPstate(12),ESPstate(0),ESPstate(1),ESPstate(2);
      }

  hopper.updateState(state);
	quat_t quat(hopper.q(6), hopper.q(3), hopper.q(4), hopper.q(5));
	hopper.v.segment(3,3) = quat._transformVector(hopper.v.segment(3,3));

  scalar_t replan = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t2).count()*1e-3;

  if(replan > p.dt){
    
    t2 = std::chrono::high_resolution_clock::now();
    
    quat_t currentQuaterion = Quaternion<scalar_t>(state(4), state(5), state(6), state(7));
    vector_3t currentEulerAngles = currentQuaterion.toRotationMatrix().eulerAngles(0, 1, 2);
    policy.updateOffsets(offsets);
    // here joystick is absolute position and yaw
    quat_des = policy.DesiredQuaternion(state(1), state(2), command(0), command(1), state(8), state(9), dist(0));
    omega_des = policy.DesiredOmega();
    u_des = policy.DesiredInputs();

          {std::lock_guard<std::mutex> lck(des_state_mtx);
      desstate[0] = quat_des.w();
      desstate[1] = quat_des.x();
      desstate[2] = quat_des.y();
      desstate[3] = quat_des.z();
      desstate[4] = omega_des(0);
      desstate[5] = omega_des(1);
      desstate[6] = omega_des(2);
      desstate[7] = u_des(1);
      desstate[8] = u_des(2);
      desstate[9] = u_des(3);
      //desstate[0] = 1;
      //desstate[1] = 0;
      //desstate[2] = 0;
      //desstate[3] = 0;
      //desstate[4] = 0;
      //desstate[5] = 0;
      //desstate[6] = 0;
      //desstate[7] = 0;
      //desstate[8] = 0;
      //desstate[9] = 0;
    }

    if (fileWrite) {
      fileHandle << std::chrono::duration_cast<std::chrono::milliseconds>(t2-tstart).count()*1e-3 << "," 
      << hopper.contact << "," << hopper.q.transpose().format(CSVFormat) << "," << hopper.v.transpose().format(CSVFormat) << "," 
      << hopper.torque.transpose().format(CSVFormat) <<"," << command.transpose().format(CSVFormat)<< "," 
      << quat_des.coeffs().transpose().format(CSVFormat) << std::endl;//<< "," << t_last_MPC << "," << sol_g.transpose().format(CSVFormat)<< "," << replan << "," << opt.elapsed_time.transpose().format(CSVFormat) << "," << opt.d_bar.cast<int>().transpose().format(CSVFormat) << "," << desstate[0] <<"," << desstate[1]<< "," << desstate[2]<< ","<< desstate[3] << "," << desstate[4] << "," << desstate[5] << "," << desstate[6] << std::endl;
    }
  }
}

}
