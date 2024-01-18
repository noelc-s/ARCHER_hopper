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
    // scalar_t MPC_dt_flight;
    // scalar_t MPC_dt_ground;
    // scalar_t MPC_dt_replan;
    // int stop_index; 
} p;

// MH
// read the yaml file and set the values for the objects p from the Structure parameters and mpc from the structure MPC_Params

void setupGains(const std::string filepath) {
    // Read gain yaml
    YAML::Node config = YAML::LoadFile(filepath);
    p.dt = config["LowLevel"]["dt"].as<scalar_t>();
}

// void setupGains(const std::string filepath, MPC::MPC_Params &mpc_p) {
//     // Read gain yaml
//     YAML::Node config = YAML::LoadFile(filepath);
//     p.dt = config["LowLevel"]["dt"].as<scalar_t>();
//     p.MPC_dt_ground = config["MPC"]["dt_ground"].as<scalar_t>();
//     p.MPC_dt_flight = config["MPC"]["dt_flight"].as<scalar_t>();
//     p.MPC_dt_replan = config["MPC"]["dt_replan"].as<scalar_t>();
//     mpc_p.N = config["MPC"]["N"].as<int>();
//     mpc_p.SQP_iter = config["MPC"]["SQP_iter"].as<int>();
//     mpc_p.discountFactor = config["MPC"]["discountFactor"].as<scalar_t>();
//     std::vector<scalar_t> tmp = config["MPC"]["stateScaling"].as<std::vector<scalar_t>>();
//     mpc_p.dt_flight= config["MPC"]["dt_flight"].as<scalar_t>();
//     mpc_p.dt_ground = config["MPC"]["dt_ground"].as<scalar_t>();
//     mpc_p.groundDuration = config["MPC"]["groundDuration"].as<scalar_t>();
//     mpc_p.heightOffset = config["MPC"]["heightOffset"].as<scalar_t>();
//     mpc_p.circle_freq = config["MPC"]["circle_freq"].as<scalar_t>();
//     mpc_p.circle_amp = config["MPC"]["circle_amp"].as<scalar_t>();
//     int nx = 20;
//     int nu = 4;
//     mpc_p.stateScaling.resize(nx);
//     mpc_p.inputScaling.resize(nu);
//     for (int i = 0; i < nx; i++)
//         mpc_p.stateScaling(i) = tmp[i];
//     tmp = config["MPC"]["inputScaling"].as<std::vector<scalar_t>>();
//     for (int i = 0; i < nu; i++)
//         mpc_p.inputScaling(i) = tmp[i];
//     mpc_p.tau_max = config["MPC"]["tau_max"].as<scalar_t>();
//     mpc_p.f_max = config["MPC"]["f_max"].as<scalar_t>();
//     mpc_p.terminalScaling = config["MPC"]["terminalScaling"].as<scalar_t>();
//     mpc_p.time_between_contacts = config["MPC"]["time_between_contacts"].as<scalar_t>();
//     mpc_p.hop_height = config["MPC"]["hop_height"].as<scalar_t>();
//     mpc_p.max_vel = config["MPC"]["max_vel"].as<scalar_t>();
// }

// Driver code
int main() {
// MH
// initialize the socket, create an object to read the mpc parameters and read the gains
    setupSocket();    
    // MPC::MPC_Params mpc_p;
    // setupGains("../config/gains.yaml", mpc_p);

    // Read yaml

    // MH
    // initializing the total states for the Hopper
    vector_t state(20);

    // MH
    // initializing the Pinnochio Model
    Hopper hopper = Hopper();


    // MH
    // q are the half the states {p, quat, theta, l}; v are the derivatives of them {pdot, omega, thetadot, ldot}
    // what are local and global wrt the original definitions? tau is for pinochhio?
    // vector_t q(11);
    // vector_t v(10);
    // vector_t q_local(11);
    // vector_t v_local(10);
    // vector_t q_global(11);
    // vector_t v_global(10);
    // vector_t tau(10);
    // Pinocchio states: pos, quat, leg, flywheeels

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
    // index for? Revisit
    // int index = 1;

    // MH
    // for discrete setup, t_last would be -1 to keep so that the planning happens for step 0:N-1?
    scalar_t t_last = -1;
    scalar_t dt_elapsed;
    // scalar_t t_last_MPC = -1;
    // scalar_t dt_elapsed_MPC;

    // MH
    // desired global angle (quat rep) is the unit quaternion; desired global rate of change of angle (omega) is [0;0;0]
    // desired flywheel and leg acceleration is [0;0;0;0]
    // quat_t quat_des = Quaternion<scalar_t>(1,0,0,0);
    // vector_3t omega_des;
    // omega_des.setZero();
    // vector_t u_des(4);
    // u_des.setZero();

    // MH
    // initializing the MPC object with 20 states, 4 inputs and the parameters (mpc_p)
    // the solutions are initialized for the local and global basis
    
    // Assuming opt.nx = 20, opt.nu = 4, opt.P.N = 20,
    // sol(20*20 + 4*19)
    // sol_g(21*20 + 4*19) 
    
    // For sanity check, the solution is a vector which is divided as such:
    // for the first 20*20 entries, each block of 20 entries are the states at steps 0, 1, 2, 3, ....., 19 (first and last state inclusive)
    // for the next 4*19 entries, each block of 4 entries are the inputs at steps, 1, 2, 3, ....., 19 (since we don't have an input at the 0th step)

    // is the extra state in the global solution a byproduct of the transformation? 
    // Which means that the sol would be in the lie algebra (and thus 3 states) vs sol_g would be in the lie group (and thus 4 states)?
    // the same logic applies for the global solution, however there is an extra step? Why?
    // MPC opt = MPC(20, 4, mpc_p);
    // vector_t sol(opt.nx*opt.p.N+opt.nu*(opt.p.N-1));
    // vector_t sol_g((opt.nx+1)*opt.p.N+opt.nu*(opt.p.N-1));
    // sol.setZero();
    // sol_g.setZero();

    // MH
    // condition variable and mutex are used for multi-threading. Ignore for now.
    // assuming, command is the global position of the robot? command_interp is the function for performing a particular action?
    
    // x_pred is a 21*2 matrix of the predicted states? Why are the dimensions such? (and not 20*1 for the predicted states at the next step)
    // u_pred is a 4*1 matrix of the predicted inputs. This makes sense, its the predicted input at the next step 
    std::condition_variable cv;
    std::mutex m;
    vector_3t command;
    vector_2t command_interp;
    std::thread userInput(getUserInput, std::ref(command), std::ref(cv), std::ref(m));
    // matrix_t x_pred(21,2);
    // matrix_t u_pred(4,1);

    // MH
    // (initializing) x_term is the terminal state in the global state (assumption), thus 21 states.
    // initializing the terminal position and quaternion for some reason. Revisit.
    // t1 and t2 for time keeping, what are they? Revisit.
    // vector_t x_term(21); x_term.setZero();
    // quat_t quat_term;
    // vector_3t pos_term;
    // std::chrono::high_resolution_clock::time_point t1;
    // std::chrono::high_resolution_clock::time_point t2;

    // MH
    // for infinity, do
    for (;;) {
        read(*new_socket, &RX_state, sizeof(RX_state));
   
        // MH
        // set t1 equal the time now
        // t1 = std::chrono::high_resolution_clock::now();
    
        // MH
        // create a map of states by reading the recieved states from the simulation
        // dt_elapsed?? dt_elapsed_MPC?? (assuming state(0) is the first state, x-position of the hopper).
        // reminder: t_last* = -1 defined previously
        Map<vector_t> state(RX_state, 20);
        dt_elapsed = state(0) - t_last;
        // dt_elapsed_MPC = state(0) - t_last_MPC;

        // MH
        // update the pinocchio object with the new state
        // define a new quaternion, z, w, x, y
        // transformVector, WHERE DO YOU LIVE? I'm guessing this is for pinocchio
        // save the current state as q0 in the local and global frame
        // replan is set to false, the next switch case does something?? Ignored for now
        hopper.updateState(state);
        // quat_t quat(hopper.q(6), hopper.q(3), hopper.q(4), hopper.q(5));
        // hopper.v.segment(3,3) = quat._transformVector(hopper.v.segment(3,3)); // Turn local omega to global omega
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
        // MH
        // if replan is false, does it solve the MPC problem?
        // if (replan) {
        //   opt.solve(hopper, sol, command, command_interp);
        //   for (int i = 0; i < opt.p.N; i++) {
        //       sol_g.segment(i*(opt.nx+1), opt.nx+1) << MPC::local2global(MPC::xik_to_qk(sol.segment(i*opt.nx,opt.nx),q0_local));
        //   }
        //   MH
        //   for the segment of the global solution for the inputs, they're just copied from the local solution
        // x_pred has 2 columns, the first column is the current state and the second column is the next state (x0 and x1), in the global basis
        // t_last_MPC is set as the first state again??
        // t2 is the time after the solution is generated
        // the terminal state is the state at the last step generated from the planner.
        // u_pred is the first input for the solution
        //   sol_g.segment((opt.nx+1)*opt.p.N,opt.nu*(opt.p.N-1)) << sol.segment((opt.nx)*opt.p.N,opt.nu*(opt.p.N-1));
        //   x_pred << MPC::local2global(MPC::xik_to_qk(sol.segment(0,20),q0_local)),MPC::local2global(MPC::xik_to_qk(sol.segment(20,20),q0_local)); 
        //   u_pred << sol.segment(opt.p.N*opt.nx,4);
        //       t_last_MPC = state(0);

        //   t2 = std::chrono::high_resolution_clock::now();
        //std::cout <<"Timing: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()*1e-6 << "[ms]" << "\n";

        //   x_term << MPC::local2global(MPC::xik_to_qk(sol.segment(opt.nx*(opt.p.N-1),20),q0_local));
        // }

        // Compute continuous time solution to discrete time problem. 
            //vector_t x_des(21);
        //hopper.css2dss(opt.Ac.block(0,0,opt.nx,opt.nx),opt.Bc.block(0,0,opt.nx,opt.nu),opt.Cc.block(0,0,opt.nx,1),state(0)-t_last_MPC,opt.Ad_,opt.Bd_,opt.Cd_);
        //x_des << MPC::local2global(MPC::Exp(opt.Ad_*sol.segment(0,20) + opt.Bd_*u_pred + opt.Cd_));
        //quat_des = Quaternion<scalar_t>(x_des(6), x_des(3), x_des(4), x_des(5));
        //omega_des << x_des(14), x_des(15),x_des(16);
        
        // MH
        // quat_des is first step of the generated solution. Similarly for omega desired
        // u_desired is the first input from the solution
        // Simply set the next waypoint as the setpoint of the low level.
        // quat_des = Quaternion<scalar_t>(x_pred(6,1), x_pred(3,1), x_pred(4,1), x_pred(5,1));
        // omega_des << x_pred(14,1), x_pred(15,1),x_pred(16,1);
        // u_des = u_pred;

        // MH
        // the terminal quaternion and positions are the extracted
        // quat_term = Quaternion<scalar_t>(x_term(6), x_term(3), x_term(4), x_term(5));
        // pos_term << x_term(0), x_term(1), x_term(2);

        // MH
        // Return to the condition, brain too fried rn
        // computeTorque takes the desired input and add a PD controller based off the error signal between the current state and desired state
        // t_last = state(0)????
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

        // MH
        // these variables are used for data logging?
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

        // TODO: Right now, the impact map takes up a dt, but that makes the timing inconsistent. Fix that.
            // Log data
            // if (fileWrite)
            // // Local
            //     //fileHandle <<state[0] << "," << hopper.contact << "," << xi_local.transpose().format(CSVFormat) << "," << hopper.torque.transpose().format(CSVFormat) << "," << t_last_MPC << "," << sol.transpose().format(CSVFormat)<< "," << replan << std::endl;
            //     // Global
            // fileHandle <<state[0] << "," << hopper.contact << "," << hopper.q.transpose().format(CSVFormat) << "," << hopper.v.transpose().format(CSVFormat) << "," << hopper.torque.transpose().format(CSVFormat) << "," << t_last_MPC << "," << sol_g.transpose().format(CSVFormat)<< "," << replan << "," << opt.elapsed_time.transpose().format(CSVFormat) << "," << opt.d_bar.cast<int>().transpose().format(CSVFormat) <<"," << command.transpose().format(CSVFormat)<< std::endl;
        
        // MH
        // the first 4 terms are the torques (flywheel and leg) applied to the hopper
        // what are the remaining terms? 

            for (int i = 0; i < 4; i++) {
                TX_torques[i] = hopper.torque[i];
            }

        // TX_torques[4] = pos_term[0];
        // TX_torques[5] = pos_term[1];
        // TX_torques[6] = pos_term[2];
        // TX_torques[7] = quat_term.w();
        // TX_torques[8] = quat_term.x();
        // TX_torques[9] = quat_term.y();
        // TX_torques[10] = quat_term.z();
        // TX_torques[11] = command_interp(0);
        // TX_torques[12] = command_interp(1);

        //	TX_torques[13] = opt.full_ref(floor(4./5*(mpc_p.N-1))*20);
        //	TX_torques[14] = opt.full_ref(floor(4./5*(mpc_p.N-1))*20+1);
        //	TX_torques[15] = opt.full_ref(floor(3./5*(mpc_p.N-1))*20);
        //	TX_torques[16] = opt.full_ref(floor(3./5*(mpc_p.N-1))*20+1);
        //	TX_torques[17] = opt.full_ref(floor(2./5*(mpc_p.N-1))*20);
        //	TX_torques[18] = opt.full_ref(floor(2./5*(mpc_p.N-1))*20+1);
        //	TX_torques[19] = opt.full_ref(floor(1./5*(mpc_p.N-1))*20);
        //	TX_torques[20] = opt.full_ref(floor(1./5*(mpc_p.N-1))*20+1);
        //	TX_torques[21] = opt.full_ref(0);
        //	TX_torques[22] = opt.full_ref(1);


        // TX_torques[13] = sol(floor(4./4*(mpc_p.N-1))*20);
        // TX_torques[14] = sol(floor(4./4*(mpc_p.N-1))*20+1);
        // TX_torques[15] = sol(floor(3./4*(mpc_p.N-1))*20);
        // TX_torques[16] = sol(floor(3./4*(mpc_p.N-1))*20+1);
        // TX_torques[17] = sol(floor(2./4*(mpc_p.N-1))*20);
        // TX_torques[18] = sol(floor(2./4*(mpc_p.N-1))*20+1);
        // TX_torques[19] = sol(floor(1./4*(mpc_p.N-1))*20);
        // TX_torques[20] = sol(floor(1./4*(mpc_p.N-1))*20+1);
        // TX_torques[21] = opt.full_ref(0);
        // TX_torques[22] = opt.full_ref(1);



            send(*new_socket, &TX_torques, sizeof(TX_torques), 0);

        // NMH: TERIMINATION CONDITION CAN BE THE ERROR BETWEEN THE ACTUAL AND DESIRED STATES?    
        // if (index == p.stop_index){
        // 	exit(2);
        // }
        // index++;
        }

}
