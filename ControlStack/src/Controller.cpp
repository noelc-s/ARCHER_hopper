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

    int *server_fd = new int;
    int *new_socket = new int;
    int valread;
    struct sockaddr_in *address = new sockaddr_in;
    int opt_socket = 1;
    int addrlen = sizeof(*address);
    scalar_t TX_torques[13+2*5] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0};
    scalar_t RX_state[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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

struct Parameters {
    std::vector<scalar_t> orientation_kp;
    std::vector<scalar_t> orientation_kd;
    scalar_t leg_kp;
    scalar_t leg_kd;
    scalar_t dt;
    scalar_t MPC_dt_flight;
    scalar_t MPC_dt_ground;
    scalar_t MPC_dt_replan;
    int predHorizon;
    int stop_index; 
    vector_t gains;
} p;

void setupGains(const std::string filepath, MPC::MPC_Params &mpc_p) {
    YAML::Node config = YAML::LoadFile(filepath);
    p.orientation_kp = config["Orientation"]["Kp"].as<std::vector<scalar_t>>();
    p.orientation_kd = config["Orientation"]["Kd"].as<std::vector<scalar_t>>();
    p.leg_kp = config["Leg"]["Kp"].as<scalar_t>();
    p.leg_kd = config["Leg"]["Kd"].as<scalar_t>();
    p.dt = config["Debug"]["dt"].as<scalar_t>();
    p.MPC_dt_ground = config["MPC"]["dt_ground"].as<scalar_t>();
    p.MPC_dt_flight = config["MPC"]["dt_flight"].as<scalar_t>();
    p.MPC_dt_replan = config["MPC"]["dt_replan"].as<scalar_t>();
    p.predHorizon = config["Debug"]["predHorizon"].as<int>();
    p.stop_index = config["Debug"]["stopIndex"].as<int>();
    p.gains.resize(8);
    p.gains << p.orientation_kp[0], p.orientation_kp[1], p.orientation_kp[2],
            p.orientation_kd[0], p.orientation_kd[1], p.orientation_kd[2],
            p.leg_kp, p.leg_kd;

        // Read gain yaml
    mpc_p.N = config["MPC"]["N"].as<int>();
    mpc_p.SQP_iter = config["MPC"]["SQP_iter"].as<int>();
    mpc_p.discountFactor = config["MPC"]["discountFactor"].as<scalar_t>();
    std::vector<scalar_t> tmp = config["MPC"]["stateScaling"].as<std::vector<scalar_t>>();
    mpc_p.dt_flight= config["MPC"]["dt_flight"].as<scalar_t>();
    mpc_p.dt_ground = config["MPC"]["dt_ground"].as<scalar_t>();
    mpc_p.groundDuration = config["MPC"]["groundDuration"].as<scalar_t>();
    mpc_p.heightOffset = config["MPC"]["heightOffset"].as<scalar_t>();
    mpc_p.circle_freq = config["MPC"]["circle_freq"].as<scalar_t>();
    mpc_p.circle_amp = config["MPC"]["circle_amp"].as<scalar_t>();
    int nx = 20;
    int nu = 4;
    mpc_p.stateScaling.resize(nx);
    mpc_p.inputScaling.resize(nu);
    for (int i = 0; i < nx; i++)
        mpc_p.stateScaling(i) = tmp[i];
    tmp = config["MPC"]["inputScaling"].as<std::vector<scalar_t>>();
    for (int i = 0; i < nu; i++)
        mpc_p.inputScaling(i) = tmp[i];
    mpc_p.tau_max = config["MPC"]["tau_max"].as<scalar_t>();
    mpc_p.f_max = config["MPC"]["f_max"].as<scalar_t>();
    mpc_p.terminalScaling = config["MPC"]["terminalScaling"].as<scalar_t>();
    mpc_p.time_between_contacts = config["MPC"]["time_between_contacts"].as<scalar_t>();
    mpc_p.hop_height = config["MPC"]["hop_height"].as<scalar_t>();
    mpc_p.max_vel = config["MPC"]["max_vel"].as<scalar_t>();
}

//
// Driver code
int main() {

    setupSocket();    
    MPC::MPC_Params mpc_p;
    setupGains("../config/gains.yaml", mpc_p);

    // Read yaml

    vector_t state(20);

    Hopper hopper = Hopper();
    //for (JointModel joint : hopper.model.joints) 
    //	    std::cout << joint << std::endl;

    vector_t q(11);
    vector_t v(10);
    vector_t q_local(11);
    vector_t v_local(10);
    vector_t q_global(11);
    vector_t v_global(10);
    vector_t tau(10);
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

    int index = 1;

    scalar_t t_last = -1;
    scalar_t dt_elapsed;
    scalar_t t_last_MPC = -1;
    scalar_t dt_elapsed_MPC;

    quat_t quat_des = Quaternion<scalar_t>(1,0,0,0);
    vector_3t omega_des;
    omega_des.setZero();
    vector_t u_des(4);
    u_des.setZero();

    MPC opt = MPC(20, 4, mpc_p);
    vector_t sol(opt.nx*opt.p.N+opt.nu*(opt.p.N-1));
    vector_t sol_g((opt.nx+1)*opt.p.N+opt.nu*(opt.p.N-1));
    sol.setZero();
    sol_g.setZero();

    //opt = MPC(20,4, mpc_p);

    std::condition_variable cv;
    std::mutex m;
    vector_3t command;
    vector_2t command_interp;
    std::thread userInput(getUserInput, std::ref(command), std::ref(cv), std::ref(m));
    matrix_t x_pred(21,2);
    matrix_t u_pred(4,1);

    vector_t x_term(21); x_term.setZero();
    quat_t quat_term;
    vector_3t pos_term;
    std::chrono::high_resolution_clock::time_point t1;
    std::chrono::high_resolution_clock::time_point t2;

    for (;;) {
        read(*new_socket, &RX_state, sizeof(RX_state));
	t1 = std::chrono::high_resolution_clock::now();

        Map<vector_t> state(RX_state, 20);
	dt_elapsed = state(0) - t_last;
	dt_elapsed_MPC = state(0) - t_last_MPC;

        hopper.updateState(state);
	quat_t quat(hopper.q(6), hopper.q(3), hopper.q(4), hopper.q(5));
	hopper.v.segment(3,3) = quat._transformVector(hopper.v.segment(3,3));
	// ^ turn the local omega to global omega
	vector_t q0(21);
	q0 << hopper.q, hopper.v;
	vector_t q0_local(21);
	q0_local = MPC::global2local(q0);
	bool replan = false;
	switch (hopper.contact>0.1) {
		case (0): {
			//replan = dt_elapsed_MPC >= p.MPC_dt_flight;
			replan = dt_elapsed_MPC >= p.MPC_dt_replan;
			break;
			  }
		case (1): {
			//replan = dt_elapsed_MPC >= p.MPC_dt_ground;
			replan = dt_elapsed_MPC >= p.MPC_dt_replan;
			break;
			  }
	}
        if (replan) {
          opt.solve(hopper, sol, command, command_interp);
	  for (int i = 0; i < opt.p.N; i++) {
            sol_g.segment(i*(opt.nx+1), opt.nx+1) << MPC::local2global(MPC::xik_to_qk(sol.segment(i*opt.nx,opt.nx),q0_local));
	  }
	  sol_g.segment((opt.nx+1)*opt.p.N,opt.nu*(opt.p.N-1)) << sol.segment((opt.nx)*opt.p.N,opt.nu*(opt.p.N-1));
	  x_pred << MPC::local2global(MPC::xik_to_qk(sol.segment(0,20),q0_local)),MPC::local2global(MPC::xik_to_qk(sol.segment(20,20),q0_local)); 
	  u_pred << sol.segment(opt.p.N*opt.nx,4);
          t_last_MPC = state(0);

	  t2 = std::chrono::high_resolution_clock::now();
	  //std::cout <<"Timing: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()*1e-6 << "[ms]" << "\n";

	  x_term << MPC::local2global(MPC::xik_to_qk(sol.segment(opt.nx*(opt.p.N-1),20),q0_local));
	}
        //vector_t x_des(21);
	//hopper.css2dss(opt.Ac.block(0,0,opt.nx,opt.nx),opt.Bc.block(0,0,opt.nx,opt.nu),opt.Cc.block(0,0,opt.nx,1),state(0)-t_last_MPC,opt.Ad_,opt.Bd_,opt.Cd_);
	//x_des << MPC::local2global(MPC::Exp(opt.Ad_*sol.segment(0,20) + opt.Bd_*u_pred + opt.Cd_));
	//quat_des = Quaternion<scalar_t>(x_des(6), x_des(3), x_des(4), x_des(5));
	//omega_des << x_des(14), x_des(15),x_des(16);
	
	quat_des = Quaternion<scalar_t>(x_pred(6,1), x_pred(3,1), x_pred(4,1), x_pred(5,1));
	omega_des << x_pred(14,1), x_pred(15,1),x_pred(16,1);
	u_des = u_pred;
	//std::cout << u_des.transpose().format(CSVFormat) << std::endl;
	// Without feedforward torque, the system goes immediately unstable
	//u_des.setZero();
	//u_des(0) = 0;

	quat_term = Quaternion<scalar_t>(x_term(6), x_term(3), x_term(4), x_term(5));
	pos_term << x_term(0), x_term(1), x_term(2);

	if (dt_elapsed > p.dt) {
		//quat_des = Quaternion<scalar_t>(1,0,0,0);
		//omega_des << 0,0,0;
		//u_des << 0,0,0,0;
        	hopper.computeTorque(quat_des, omega_des, 0.1, u_des);
		t_last = state(0);
	}

	vector_t v_global(6);
	vector_t v_local(6);
	vector_t x_global(21);
	vector_t x_local(21);
	vector_t xi_local(21);
	x_global << hopper.q, hopper.v;
	x_local = MPC::global2local(x_global);
	xi_local = MPC::Log(x_local);
	v_global = hopper.v.segment(0,6);
	v_local = x_local.segment(11,6);

	//std::cout << "v_global: " << v_global.transpose() << std::endl;
	//std::cout << "v_local: " << v_local.transpose() << std::endl;
	//std::cout << "----------------------" << std::endl;

	// TODO: Right now, the impact map takes up a dt, but that makes the timing inconsistent. Fix that.

	///////////////////////////////////////////////////
	////////////////////// Checks /////////////////////
	///////////////////////////////////////////////////
	//vector_t s_kp1(20);
        //vector_t s(20);
	//vector_t x_kp1(21);
	//vector_t x_k(21);
	//matrix_t Ac(20,20); Ac.setZero();
        //matrix_t Bc(20,4); Bc.setZero();
        //matrix_t Cc(20,1); Cc.setZero();
        //matrix_t Ad(20,20); Ad.setZero();
        //matrix_t Bd(20,4); Bd.setZero();
        //matrix_t Cd(20,1); Cd.setZero();
        //vector_t x(21);
	//quat_t quat;
	//manif::SO3<scalar_t> quat_;
	//manif::SO3Tangent<scalar_t> xi;
	//manif::SO3Tangent<scalar_t> xi_kp1;
	//matrix_t J(6,10); J.setZero();
	//vector_3t p_l, v_l, p_g, v_g;
	//matrix_3t Rq;
	//scalar_t qw,qx,qy,qz;
	//vector_3t p;
	//vector_3t q_dot;
	
        //q << hopper.pos, hopper.quat.coeffs(), hopper.leg_pos, 0, 0, 0;
        //v << hopper.vel, hopper.omega, hopper.leg_vel, hopper.wheel_vel;
        //tau << 0, 0, 0, 0, 0, 0, hopper.torque;
	
	// Does the Exp map cancel the Log map?
	// Yes.
	//quat_t quat(1,0,0,0);
	//quat_t quat(0.7071,0,0,0.7071);
	//quat.normalize();
        //auto quat_ = manif::SO3<scalar_t>(hopper.quat);
	//std::cout << "Quat: \t\t" << hopper.quat.coeffs().transpose() << std::endl;
        //auto xi = quat_.log();
	//std::cout << "log(Quat):\t " << xi << std::endl;
	//std::cout << "exp(log(Quat)): " << xi.exp().quat().coeffs().transpose() << std::endl;
	//std::cout << "------------------------------" << std::endl;

	// Do the static dynamics make sense?
	// Yes.
	//quat = Eigen::Quaternion<scalar_t>(1,0,0,0);
        //q << 0, 0, 1, quat.coeffs(), 0, 0, 0, 0;
        //v << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        //tau << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	//d = ground;
	//std::cout << "Static Dynamics: " << hopper.f(q,v,tau,d).transpose() << std::endl;
        //matrix_t A(20,20);
        //matrix_t B(20,4);
        //matrix_t C(20,1);
        //hopper.Df(q,v,tau,d,A,B,C);
	//vector_t x_kp1(20);
	//x_kp1 = hopper.oneStepPredict(q,v,tau,0.05,flight);


	// Do the frame transformations cancel each other out?
	// Yes. That does not mean that they are right though.
	//p_g << q.segment(0,3);
	//p_l << hopper.quat.inverse()._transformVector(p_g);
	//q_local << p_l, hopper.quat.coeffs(),q.segment(7,4);
	//qw = hopper.quat.w();
	//qx = hopper.quat.x();
	//qy = hopper.quat.y();
	//qz = hopper.quat.z();
	//Rq << pow(qw,2)+pow(qx,2)-pow(qy,2)-pow(qz,2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy),
	//   2*(qx*qy+qw*qz), pow(qw,2)-pow(qx,2)+pow(qy,2)-pow(qz,2),2*(qy*qz-qw*qx),
	//   2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), pow(qw,2)-pow(qx,2)-pow(qy,2)+pow(qz,2);
	//v_g = v.segment(0,3);
	//v_l << Rq.transpose()*v_g + Hopper::cross(Rq.transpose()*p_g)*(1./2.*v.segment(3,3));
	//v_local << v_l, v.segment(3,7);
	//std::cout << "----------- Frame position transformations ---------" << std::endl;
	//std::cout << "Global pos: " << p_g.transpose() << std::endl;
	//std::cout << "Local pos: " << p_l.transpose() << std::endl;
	//std::cout << "Global pos: " << hopper.quat._transformVector(p_l).transpose() << std::endl;
	//std::cout << "----------- Frame velocity transformations ---------" << std::endl;
	//std::cout << "Global vel: " << v_g.transpose() << std::endl;
	//std::cout << "Local vel: " << v_l.transpose() << std::endl;
	// Based on this: http://jamessjackson.com/lie_algebra_tutorial/09-lie_group_tables/
	// Still do not know why it is positive and cross of the whole thing
	//std::cout << "Global vel: " <<  (Rq*v_l - Rq*Hopper::cross(p_l)*(1./2.*v.segment(3,3))).transpose() << std::endl;
	//std::cout << "State: " << state.transpose() << std::endl;
	//std::cout << "d: " << d << std::endl;
	//std::cout << "q: " << q.transpose() << std::endl;
	//std::cout << "v: " << v.transpose() << std::endl;
	//std::cout << "tau: " << tau.transpose() << std::endl;
	
	// Do the implemented functions cancel each other out?
	// Yes.
	//vector_t x_local(21);
	//vector_t x_global(21);
	//x_global << hopper.q, hopper.v;
	//x_local = MPC::global2local(x_global);
	//std::cout << "x_global: " << x_global.transpose() << std::endl;
	//std::cout << "x_local: " << x_local.transpose() << std::endl;
	//std::cout << "x_global reset: " << MPC::local2global(x_local).transpose() << std::endl;
	//std::cout << "-------------" << std::endl;
	
	// Does the discrete update map make sense?
	// Yes.
	//std::cout << "x_minus: " << x_l.transpose() << std::endl;
	//std::cout << "x_plus: " << hopper.delta_f(x_l.segment(0,11), x_l.segment(11,10), flight).transpose() << std::endl;

        //FileHandleDebug << x.transpose().format(CSVFormat) << std::endl;
	//X_kp1 << q_local, v_local;
	//X_k << q, v;
	//Quat = hopper.quat;
	//Vector_t x_intermediate(21);
	//For(int i = 0; i < predHorizon; i ++) {
	//  x_kp1 = opt.oneStepPredict(hopper, sol_g.segment(0,11), sol_g.segment(11,10), tau, dt, d);
        //  fileHandleDebug << x_kp1.transpose().format(CSVFormat) << std::endl;
	//  //x_intermediate = MPC::local2global(x_kp1);
	//  q << x_kp1.segment(0,11);
	//  v << x_kp1.segment(11,10);
	//}

	
	// notes
	// - Should make quat_to_xi function
	
	// Printing Dynamics matrices to make sure they look right
	//std::cout << "Ac: " << std::endl << opt.Ac.block(0,0,opt.nx,opt.nx) <<std::endl;
	//std::cout << "--------------------------------------" << std::endl;
	//std::cout << "Ad: " << std::endl << opt.Ad_ <<std::endl;
	//std::cout << "--------------------------------------" << std::endl;
	//std::cout << "Bc: " << std::endl << opt.Bc.block(0,0,opt.nx,opt.nu) <<std::endl;
	//std::cout << "--------------------------------------" << std::endl;
	//std::cout << "Bd: " << std::endl << opt.Bd_ <<std::endl;
	//std::cout << "--------------------------------------" << std::endl;
	//std::cout << "Cc: " << std::endl << opt.Cc.block(0,0,opt.nx,1) <<std::endl;
	//std::cout << "--------------------------------------" << std::endl;
	//std::cout << "Cd: " << std::endl << opt.Cd_ <<std::endl;
	//std::cout << "--------------------------------------" << std::endl;

        // Log data
        if (fileWrite)
		// Local
            //fileHandle <<state[0] << "," << hopper.contact << "," << xi_local.transpose().format(CSVFormat) << "," << hopper.torque.transpose().format(CSVFormat) << "," << t_last_MPC << "," << sol.transpose().format(CSVFormat)<< "," << replan << std::endl;
            // Global
	    fileHandle <<state[0] << "," << hopper.contact << "," << hopper.q.transpose().format(CSVFormat) << "," << hopper.v.transpose().format(CSVFormat) << "," << hopper.torque.transpose().format(CSVFormat) << "," << t_last_MPC << "," << sol_g.transpose().format(CSVFormat)<< "," << replan << "," << opt.elapsed_time.transpose().format(CSVFormat) << "," << opt.d_bar.cast<int>().transpose().format(CSVFormat) <<"," << command.transpose().format(CSVFormat)<< std::endl;

        for (int i = 0; i < 4; i++) {
            TX_torques[i] = hopper.torque[i];
        }

	TX_torques[4] = pos_term[0];
	TX_torques[5] = pos_term[1];
	TX_torques[6] = pos_term[2];
	TX_torques[7] = quat_term.w();
	TX_torques[8] = quat_term.x();
	TX_torques[9] = quat_term.y();
	TX_torques[10] = quat_term.z();
	TX_torques[11] = command_interp(0);
	TX_torques[12] = command_interp(1);

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


	TX_torques[13] = sol(floor(4./4*(mpc_p.N-1))*20);
	TX_torques[14] = sol(floor(4./4*(mpc_p.N-1))*20+1);
	TX_torques[15] = sol(floor(3./4*(mpc_p.N-1))*20);
	TX_torques[16] = sol(floor(3./4*(mpc_p.N-1))*20+1);
	TX_torques[17] = sol(floor(2./4*(mpc_p.N-1))*20);
	TX_torques[18] = sol(floor(2./4*(mpc_p.N-1))*20+1);
	TX_torques[19] = sol(floor(1./4*(mpc_p.N-1))*20);
	TX_torques[20] = sol(floor(1./4*(mpc_p.N-1))*20+1);
	TX_torques[21] = opt.full_ref(0);
	TX_torques[22] = opt.full_ref(1);



        send(*new_socket, &TX_torques, sizeof(TX_torques), 0);
	if (index == p.stop_index){
		exit(2);
	}
	index++;
    }

}
