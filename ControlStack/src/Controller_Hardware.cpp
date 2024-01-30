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

#include "pinocchio/algorithm/jacobian.hpp"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/String.h"

#include <manif/manif.h>

#include "../inc/Hopper.h"
#include "../inc/Types.h"
// #include "../inc/MPC.h"
#include "../inc/Policy.h"

#define MAXLINE 1000
#define MAX 48
#define STATE_SIZE 40
#define MSG_SIZE 48+4
#define PORT 8888
#define SA struct sockaddr

using namespace Eigen;
using namespace Hopper_t;
using namespace pinocchio;


const static IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

int sockfd, connfd;
char buff[60];
char send_buff[46];
float states[13];
vector_t ESPstate(13);
float desstate[10];
std::mutex state_mtx;
std::mutex des_state_mtx;

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

struct Parameters {
    std::vector<scalar_t> orientation_kp;
    std::vector<scalar_t> orientation_kd;
    scalar_t leg_kp;
    scalar_t leg_kd;
    scalar_t dt;
    // scalar_t MPC_dt_flight;
    // scalar_t MPC_dt_ground;
    // scalar_t MPC_dt_replan;
    scalar_t frameOffset;
    scalar_t markerOffset;
    int predHorizon;
    int stop_index; 
    vector_t gains;
    std::vector<scalar_t> p0;
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
    // p.MPC_dt_ground = config["MPC"]["dt_ground"].as<scalar_t>();
    // p.MPC_dt_flight = config["MPC"]["dt_flight"].as<scalar_t>();
    // p.MPC_dt_replan = config["MPC"]["dt_replan"].as<scalar_t>();
    // p.frameOffset = config["MPC"]["frameOffset"].as<scalar_t>();
    // p.markerOffset = config["MPC"]["markerOffset"].as<scalar_t>();
    p.predHorizon = config["Debug"]["predHorizon"].as<int>();
    p.stop_index = config["Debug"]["stopIndex"].as<int>();
    p.p0 = config["Simulator"]["p0"].as<std::vector<scalar_t>>();
    p.gains.resize(8);
    p.gains << p.orientation_kp[0], p.orientation_kp[1], p.orientation_kp[2],
            p.orientation_kd[0], p.orientation_kd[1], p.orientation_kd[2],
            p.leg_kp, p.leg_kd;

        // Read gain yaml
    // mpc_p.N = config["MPC"]["N"].as<int>();
    // mpc_p.SQP_iter = config["MPC"]["SQP_iter"].as<int>();
    // mpc_p.discountFactor = config["MPC"]["discountFactor"].as<scalar_t>();
    // std::vector<scalar_t> tmp = config["MPC"]["stateScaling"].as<std::vector<scalar_t>>();
    // mpc_p.dt_flight= config["MPC"]["dt_flight"].as<scalar_t>();
    // mpc_p.dt_ground = config["MPC"]["dt_ground"].as<scalar_t>();
    // mpc_p.groundDuration = config["MPC"]["groundDuration"].as<scalar_t>();
    // mpc_p.heightOffset = config["MPC"]["heightOffset"].as<scalar_t>();
    // mpc_p.circle_freq = config["MPC"]["circle_freq"].as<scalar_t>();
    // mpc_p.circle_amp = config["MPC"]["circle_amp"].as<scalar_t>();
    // int nx = 20;
    // int nu = 4;
    // mpc_p.stateScaling.resize(nx);
    // mpc_p.inputScaling.resize(nu);
    // for (int i = 0; i < nx; i++)
    //     mpc_p.stateScaling(i) = tmp[i];
    // tmp = config["MPC"]["inputScaling"].as<std::vector<scalar_t>>();
    // for (int i = 0; i < nu; i++)
    //     mpc_p.inputScaling(i) = tmp[i];
    // mpc_p.tau_max = config["MPC"]["tau_max"].as<scalar_t>();
    // mpc_p.f_max = config["MPC"]["f_max"].as<scalar_t>();
    // mpc_p.terminalScaling = config["MPC"]["terminalScaling"].as<scalar_t>();
    // mpc_p.time_between_contacts = config["MPC"]["time_between_contacts"].as<scalar_t>();
    // mpc_p.hop_height = config["MPC"]["hop_height"].as<scalar_t>();

}

volatile bool ESP_initialized = false;

void getStateFromESP() {
  while(1) {

  //receive string states, ESP8266 -> PC
  read(sockfd, buff, sizeof(buff));
  char oneAdded[8];
  memcpy(oneAdded, buff+52, 8*sizeof(char));
  for (int i = 0; i < 8; i++) {
    for (int j = 1; j < 8; j++) {
      if(oneAdded[i] & (1 << (8-j))) {
        buff[i*7+(j-1)] = 0;
      }
    }
  }

  memcpy(&states, buff, 13*sizeof(float));
  quat_t quat_a = quat_t(states[6], states[7], states[8], states[9]);
  quat_a.normalize();
  {std::lock_guard<std::mutex> lck(state_mtx);
  ESPstate << states[0], states[1], states[2], states[3], states[4], states[5], quat_a.w(), quat_a.x(), quat_a.y(), quat_a.z(), states[10], states[11], states[12];
  }

  // encode send_buff
  {std::lock_guard<std::mutex> lck(des_state_mtx);
  memcpy(send_buff, desstate, 10*4);
  }
  for (int i = 0; i < 6; i++) {
      char oneAdded = 0b00000001;
      for (int j = 1; j < 8; j++){
        if (send_buff[i*7+(j-1)] == 0b00000000) {
          send_buff[i*7+(j-1)] = 0b00000001;
          oneAdded += (1 << (8-j));
        }
      }
      memcpy(&send_buff[40+i], &oneAdded, 1);
  }

  write(sockfd, send_buff, sizeof(send_buff));
  ESP_initialized = true;
  }

}

void setupSocket() {
        // socket stuff
        struct sockaddr_in servaddr, cli;

        // socket create and verification
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd == -1) {
                printf("socket creation failed...\n");
                exit(0);
        }
        else
                printf("Socket successfully created..\n");
        bzero(&servaddr, sizeof(servaddr));

        // assign IP, PORT
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = inet_addr("192.168.1.4");
        servaddr.sin_port = htons(PORT);

        // connect the client socket to server socket
        if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) {
                printf("connection with the server failed...\n");
                exit(0);
        }
        else
                printf("connected to the server..\n");
        sleep(1);
}

//
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
    std::thread thread_object(getStateFromESP);

    std::chrono::high_resolution_clock::time_point t1;
    std::chrono::high_resolution_clock::time_point t2;
    std::chrono::high_resolution_clock::time_point tstart;

    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////

    // Read yaml
    // MPC::MPC_Params mpc_p;
    setupGains("../config/gains_hardware.yaml");

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

    // Set up Data logging
    //bool fileWrite = true;
    //std::string dataLog = "../data/data.csv";
    //std::string predictionLog = "../data/prediction.csv";
    //std::ofstream fileHandle;
    //fileHandle.open(dataLog);
    //fileHandle << "t,x,y,z,q_w,q_x,q_y,q_z,x_dot,y_dot,z_dot,w_1,w_2,w_3,contact,l,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,z_acc";
    //std::ofstream fileHandleDebug;
    //fileHandleDebug.open(predictionLog);
    //fileHandleDebug << "t,x,y,z,q_w,q_x,q_y,q_z,x_dot,y_dot,z_dot,w_1,w_2,w_3,contact,l,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,z_acc";

    int index = 1;

    scalar_t t_last = -1;
    scalar_t dt_elapsed;
    // scalar_t t_last_MPC = -1;
    // scalar_t t_log_MPC = -1;
    // scalar_t dt_elapsed_MPC;

    quat_t quat_des = Quaternion<scalar_t>(1,0,0,0);
    vector_3t omega_des;
    omega_des.setZero();
    vector_t u_des(4);
    u_des.setZero();

    // MPC opt = MPC(20, 4, mpc_p);
    // vector_t sol(opt.nx*opt.p.N+opt.nu*(opt.p.N-1));
    // vector_t sol_g((opt.nx+1)*opt.p.N+opt.nu*(opt.p.N-1));
    // sol.setZero();
    // sol_g.setZero();

    //opt = MPC(20,4, mpc_p);

    std::condition_variable cv;
    std::mutex m;
    vector_3t command;
    vector_2t command_interp;
    std::thread userInput(getUserInput, std::ref(command), std::ref(cv), std::ref(m));
    // matrix_t x_pred(21,2);
    // matrix_t u_pred(4,1);

    // vector_t x_term(21); x_term.setZero();
    // quat_t quat_term;
    // vector_3t pos_term;

    vector_3t last_state;


    // ROS stuff
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/vrpn_client_node/hopper/pose", 200, chatterCallback);

    while(!ESP_initialized) {};

    vector_3t current_vel, previous_vel;
    scalar_t dt = 1;
    std::chrono::high_resolution_clock::time_point last_t_state_log;

    tstart = std::chrono::high_resolution_clock::now();
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


  //       // time[1], pos[3], quat[4], vel[3], omega[3], contact[1], leg (pos,vel)[2], flywheel speed
  //       //ESPstate: wheel speed, omega, quat;
	
	// dt_elapsed = state(0) - t_last;
	// dt_elapsed_MPC = state(0) - t_last_MPC;
	

  //       hopper.updateState(state);
	// quat_t quat(hopper.q(6), hopper.q(3), hopper.q(4), hopper.q(5));
	// hopper.v.segment(3,3) = quat._transformVector(hopper.v.segment(3,3));
	// // ^ turn the local omega to global omega
	// vector_t q0(21);
	// q0 << hopper.q, hopper.v;
	// vector_t q0_local(21);
	// q0_local = MPC::global2local(q0);
	// bool replan = false;
	// switch (hopper.contact>0.1) {
	// 	case (0): {
	// 		//replan = dt_elapsed_MPC >= p.MPC_dt_flight;
	// 		replan = dt_elapsed_MPC >= p.MPC_dt_replan;
	// 		break;
	// 		  }
	// 	case (1): {
	// 		//replan = dt_elapsed_MPC >= p.MPC_dt_ground;
	// 		replan = dt_elapsed_MPC >= p.MPC_dt_replan;
	// 		break;
	// 		  }
	// }
  //       if (replan) {
	//   t2 = std::chrono::high_resolution_clock::now();
  //         t_last_MPC = std::chrono::duration_cast<std::chrono::milliseconds>(t2-tstart).count()*1e-3;
	//   previous_vel = current_vel;
	//   last_state << state(1), state(2), state(3);
	//   last_t_state_log = t1;

	//   //std::cout << "State: " << state.transpose().format(CSVFormat) << std::endl;
	//   //std::cout << "dt: " << dt << std::endl;
  //         opt.solve(hopper, sol, command, command_interp);
	//   for (int i = 0; i < opt.p.N; i++) {
  //           sol_g.segment(i*(opt.nx+1), opt.nx+1) << MPC::local2global(MPC::xik_to_qk(sol.segment(i*opt.nx,opt.nx),q0_local));
	//   }
	//   sol_g.segment((opt.nx+1)*opt.p.N,opt.nu*(opt.p.N-1)) << sol.segment((opt.nx)*opt.p.N,opt.nu*(opt.p.N-1));
	//   x_pred << MPC::local2global(MPC::xik_to_qk(sol.segment(20,20),q0_local)),MPC::local2global(MPC::xik_to_qk(sol.segment(40,20),q0_local)); 
	//   u_pred << sol.segment(opt.p.N*opt.nx+4,4);

	//   t2 = std::chrono::high_resolution_clock::now();
	//   //std::cout <<"Timing: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()*1e-6 << "[ms]" << "\n";

  //         t_log_MPC = std::chrono::duration_cast<std::chrono::milliseconds>(t2-tstart).count()*1e-3;

	//   x_term << MPC::local2global(MPC::xik_to_qk(sol.segment(opt.nx*(opt.p.N-1),20),q0_local));

	// }
        //vector_t x_des(21);
	//hopper.css2dss(opt.Ac.block(0,0,opt.nx,opt.nx),opt.Bc.block(0,0,opt.nx,opt.nu),opt.Cc.block(0,0,opt.nx,1),state(0)-t_last_MPC,opt.Ad_,opt.Bd_,opt.Cd_);
	//x_des << MPC::local2global(MPC::Exp(opt.Ad_*sol.segment(0,20) + opt.Bd_*u_pred + opt.Cd_));
	//quat_des = Quaternion<scalar_t>(x_des(6), x_des(3), x_des(4), x_des(5));
	//omega_des << x_des(14), x_des(15),x_des(16);
	
	// quat_des = Quaternion<scalar_t>(x_pred(6,1), x_pred(3,1), x_pred(4,1), x_pred(5,1));
	// omega_des << x_pred(14,1), x_pred(15,1),x_pred(16,1);
	// u_des = u_pred;

  Policy policy = Policy();
  
  quat_des = policy.DesiredQuaternion(state(1), state(2), command(0), command(1), state(8), state(9));
  omega_des = policy.DesiredOmega();
  u_des = policy.DesiredInputs();

	// quat_term = Quaternion<scalar_t>(x_term(6), x_term(3), x_term(4), x_term(5));
	// pos_term << x_term(0), x_term(1), x_term(2);

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

	//if (dt_elapsed > p.dt) {
	//	//quat_des = Quaternion<scalar_t>(1,0,0,0);
        //	hopper.computeTorque(quat_des, omega_des, 0.1, u_des);
	//	t_last = state(0);
	//}

	// vector_t v_global(6);
	// vector_t v_local(6);
	// vector_t x_global(21);
	// vector_t x_local(21);
	// vector_t xi_local(21);
	// x_global << hopper.q, hopper.v;
	// x_local = MPC::global2local(x_global);
	// xi_local = MPC::Log(x_local);
	// v_global = hopper.v.segment(0,6);
	// v_local = x_local.segment(11,6);
  //       // Log data
	// t2 = std::chrono::high_resolution_clock::now();
	// if (replan)
  //   if (fileWrite)
	//     fileHandle << std::chrono::duration_cast<std::chrono::milliseconds>(t2-tstart).count()*1e-3 << "," << hopper.contact << "," << hopper.q.transpose().format(CSVFormat) << "," << hopper.v.transpose().format(CSVFormat) << "," << hopper.torque.transpose().format(CSVFormat) << "," << t_last_MPC << "," << sol_g.transpose().format(CSVFormat)<< "," << replan << "," << opt.elapsed_time.transpose().format(CSVFormat) << "," << opt.d_bar.cast<int>().transpose().format(CSVFormat) << "," << desstate[0] <<"," << desstate[1]<< "," << desstate[2]<< ","<< desstate[3] << "," << desstate[4] << "," << desstate[5] << "," << desstate[6] << std::endl;

    }

}
