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

#include "Hopper.h"
#include "Graph.h"
#include "Controller.h"

using namespace Eigen;
using namespace Hopper_t;
using namespace pinocchio;

// Driver code
int main(int argc, char *argv[]) {

  ///////////// Variable Initialization ///////////////////////////
  /////////////////////////////////////////////////////////////////
  // Socket related variables
  int *new_socket = new int;
  int *server_fd = new int;
  struct sockaddr_in* address = new sockaddr_in;
  std::unique_ptr<uint16_t> port;
  if (argc < 2) {
    port.reset(new uint16_t(8080));
  } else {
    port.reset(new uint16_t(static_cast<uint16_t>(std::stoul(argv[1]))));
  }

  // MPC related variables
  Hopper hopper = Hopper();
  Controller controller;
  MPC::MPC_Params mpc_p;
  setupGains("../config/gains.yaml", mpc_p);
  MPC opt = MPC(20, 4, mpc_p);
  vector_t sol(opt.nx*opt.p.N+opt.nu*(opt.p.N-1));
  vector_t sol_g((opt.nx+1)*opt.p.N+opt.nu*(opt.p.N-1));
  sol.setZero();
  sol_g.setZero();
  vector_t x_term(21); x_term.setZero();
  quat_t quat_term;
  vector_3t pos_term;
  matrix_t x_pred(21,2);
  matrix_t u_pred(4,1);

  // State variables
  vector_t state(20);
  vector_t q(11);
  vector_t v(10);
  vector_t q_local(11);
  vector_t v_local(10);
  vector_t q_global(11);
  vector_t v_global(10);
  vector_t tau(10);
  // Pinocchio states: pos, quat, leg, flywheeels

  // Data logging related variables
  bool fileWrite = true;
  std::string dataLog = "../data/data.csv";
  std::string predictionLog = "../data/prediction.csv";
  std::ofstream fileHandle;
  fileHandle.open(dataLog);
  fileHandle << "t,x,y,z,q_w,q_x,q_y,q_z,x_dot,y_dot,z_dot,w_1,w_2,w_3,contact,l,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,z_acc";
  std::ofstream fileHandleDebug;
  fileHandleDebug.open(predictionLog);
  fileHandleDebug << "t,x,y,z,q_w,q_x,q_y,q_z,x_dot,y_dot,z_dot,w_1,w_2,w_3,contact,l,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,z_acc";

  // Loop variables
  int index = 1;
  std::chrono::high_resolution_clock::time_point t1;
  std::chrono::high_resolution_clock::time_point t2;
  
  // Mutex
  std::condition_variable cv;
  std::mutex m;

  // Command and disturbance variables
  vector_3t command;
  vector_2t dist;
  vector_2t command_interp;
  std::thread userInput(getJoystickInput, std::ref(command), std::ref(dist), std::ref(cv), std::ref(m));

  //////////////////////// Trajectory Test ////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  vector_array_t waypts;
  scalar_array_t times;
 
  vector_t vec(12);
  vec.setZero();

  vec.segment(0,2) << 0,0;
  waypts.push_back(vec);
  vec.segment(0,2) << -1,0;
  waypts.push_back(vec);
  vec.segment(0,2) << 1,0;
  waypts.push_back(vec);
  vec.segment(0,2) << -1,1;
  waypts.push_back(vec);
  vec.segment(0,2) << 0,0;
  waypts.push_back(vec);
  vec.segment(0,2) << 1.9,0;
  waypts.push_back(vec);
  vec.segment(0,2) << 3.5,0;
  waypts.push_back(vec);

  for (int i=0; i<waypts.size(); i++) {
    times.push_back(1.* (float) i);
  }

  Traj trajectory = {waypts,times, waypts.size()};
  Bezier_T tra(trajectory, 1.0);

  /////////////////////// Graph Test ///////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////
 // // vertices
 // vertex_array_t V_list;
 // 
 // matrix_t C_(3,2);
 // vector_t d_(3);
 // vector_t x_(2);

 // Polytope h;
 // Vertex v_;

 // C_ << 0.9, 0.8,
 //      10., -5.3,
 //      -6.7, 1.0;
 // d_ << 0.8, 0. ,1;
 // x_ << 1,1;
 // h = {C_, d_};
 // v_ = {x_,h};
 // V_list.push_back(v_);

 // C_ << 1, 0.9,
 //      11., -5.,
 //      -6.6, 1.1;
 // d_ << 0.7, 0.1, 10 ;
 // x_ << 2,2;
 // h = {C_, d_};
 // v_ = {x_,h};
 // V_list.push_back(v_);
 // 
 // // edges
 // Edge e;
 // e = {V_list[0], V_list[1], h, (scalar_t) 1.0};
 // edge_array_t E_list;
 // E_list.push_back(e);

 // //construct the graph
 // Graph graph(V_list, E_list);
 // vector_t x(2);
 // x << .2419,.519;
 // std::cout << graph.isInPolytope(x, h) << std::endl;

 // std::cout << "number vertices in G = " << graph.G.V.size() << std::endl;
 // std::cout << "number edges in G = " << graph.G.E.size() << std::endl;

 //////////////////////// Main Loop /////////////////////////////////
 ////////////////////////////////////////////////////////////////////
    setupSocket(new_socket, server_fd, address, *port);    

    
    // for counting number of hops based on z velocity
    hopper.num_hops = 0;
    int sign_flips =0;
    bool pos_sign = hopper.vel(2) > 0;
    bool pos_sign_check;



    quat_t quat_des = Quaternion<scalar_t>(1,0,0,0);
    vector_3t omega_des;
    omega_des.setZero();
    vector_t u_des(4);
    u_des.setZero();

    read(*new_socket, &RX_state, sizeof(RX_state));

    for (;;) {
	// If the reset flag has been set, return the program state to running
        if (controller.programState_ == RESET) {
                controller.programState_ = RUNNING;
        }
	t1 = std::chrono::high_resolution_clock::now();

        Map<vector_t> state(RX_state, 20);
	controller.dt_elapsed = state(0) - controller.t_last;
	controller.dt_elapsed_MPC = state(0) - controller.t_last_MPC;

        hopper.updateState(state);
	quat_t quat(hopper.q(6), hopper.q(3), hopper.q(4), hopper.q(5));
	hopper.v.segment(3,3) = quat._transformVector(hopper.v.segment(3,3)); // Turn local omega to global omega
	vector_t q0(21);
	q0 << hopper.q, hopper.v;
	vector_t q0_local(21);
	q0_local = MPC::global2local(q0);
	bool replan = false;
	switch (hopper.contact>0.1) {
		case (0): {
			//replan = controller.dt_elapsed_MPC >= p.MPC_dt_flight;
			replan = controller.dt_elapsed_MPC >= p.MPC_dt_replan;
			break;
			  }
		case (1): {
			//replan = controller.dt_elapsed_MPC >= p.MPC_dt_ground;
			replan = controller.dt_elapsed_MPC >= p.MPC_dt_replan;
			break;
			  }
	}
        if (replan) {
          opt.solve(hopper, sol, command, command_interp, &tra); ///////////////////////////////////
	  for (int i = 0; i < opt.p.N; i++) {
            sol_g.segment(i*(opt.nx+1), opt.nx+1) << MPC::local2global(MPC::xik_to_qk(sol.segment(i*opt.nx,opt.nx),q0_local));
	  }
	  sol_g.segment((opt.nx+1)*opt.p.N,opt.nu*(opt.p.N-1)) << sol.segment((opt.nx)*opt.p.N,opt.nu*(opt.p.N-1));
	  x_pred << MPC::local2global(MPC::xik_to_qk(sol.segment(0,20),q0_local)),MPC::local2global(MPC::xik_to_qk(sol.segment(20,20),q0_local)); 
	  u_pred << sol.segment(opt.p.N*opt.nx,4);
          controller.t_last_MPC = state(0);

	  t2 = std::chrono::high_resolution_clock::now();
	  //std::cout <<"Timing: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()*1e-6 << "[ms]" << "\n";

	  x_term << MPC::local2global(MPC::xik_to_qk(sol.segment(opt.nx*(opt.p.N-1),20),q0_local));
	}

	// Compute continuous time solution to discrete time problem. 
        //vector_t x_des(21);
	//hopper.css2dss(opt.Ac.block(0,0,opt.nx,opt.nx),opt.Bc.block(0,0,opt.nx,opt.nu),opt.Cc.block(0,0,opt.nx,1),state(0)-controller.t_last_MPC,opt.Ad_,opt.Bd_,opt.Cd_);
	//x_des << MPC::local2global(MPC::Exp(opt.Ad_*sol.segment(0,20) + opt.Bd_*u_pred + opt.Cd_));
	//quat_des = Quaternion<scalar_t>(x_des(6), x_des(3), x_des(4), x_des(5));
	//omega_des << x_des(14), x_des(15),x_des(16);
	
	// Simply set the next waypoint as the setpoint of the low level.
	quat_des = Quaternion<scalar_t>(x_pred(6,1), x_pred(3,1), x_pred(4,1), x_pred(5,1));
	omega_des << x_pred(14,1), x_pred(15,1),x_pred(16,1);
	u_des = u_pred;

	quat_term = Quaternion<scalar_t>(x_term(6), x_term(3), x_term(4), x_term(5));
	pos_term << x_term(0), x_term(1), x_term(2);

	if (controller.dt_elapsed > p.dt) {
		// Uncomment below to isolate the low level controller for testing
		//quat_des = Quaternion<scalar_t>(1,0,0,0);
		//omega_des << 0,0,0;
		//u_des << 0,0,0,0;

        	hopper.computeTorque(quat_des, omega_des, 0.1, u_des);
		controller.t_last = state(0);
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

	// TODO: Right now, the impact map takes up a dt, but that makes the timing inconsistent. Fix that.
        // Log data
        if (fileWrite)
	    // Local
            //fileHandle <<state[0] << "," << hopper.contact << "," << xi_local.transpose().format(CSVFormat) << "," << hopper.torque.transpose().format(CSVFormat) << "," << controller.t_last_MPC << "," << sol.transpose().format(CSVFormat)<< "," << replan << std::endl;
            // Global
	    fileHandle <<state[0] << "," << hopper.contact << "," << hopper.q.transpose().format(CSVFormat) << "," << hopper.v.transpose().format(CSVFormat) << "," << hopper.torque.transpose().format(CSVFormat) << "," << controller.t_last_MPC << "," << sol_g.transpose().format(CSVFormat)<< "," << replan << "," << opt.elapsed_time.transpose().format(CSVFormat) << "," << opt.d_bar.cast<int>().transpose().format(CSVFormat) <<"," << command.transpose().format(CSVFormat)<< std::endl;

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
	TX_torques[12] = command_interp(1); // why do you need command_interp here?

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

	TX_torques[23] = dist(0);
	TX_torques[24] = dist(1);

	// reset MPC
	// NOTICE: time gets reset, so this loops. Do not use state(0) [time] as a flag,
	// becuase it will be reset by the simulator
        if (state(0) > 1.) {
	  //controller.startSimulation();
	  //controller.stopSimulation();
	  vector_t x0(12);
	  x0 << 0,0,1.5,0,0,0,0,0,0,0,0,0;
	  controller.resetSimulation(x0, TX_torques);
        }

	TX_torques[25] = (scalar_t) controller.programState_;



  // count number of hops based on changing directions of z velocity
  pos_sign_check = hopper.vel(2) > 0;
  if (pos_sign != pos_sign_check) {
    pos_sign = pos_sign_check;
    sign_flips ++;
    if (sign_flips % 2 ==0)
      hopper.num_hops++;
  }

  std::cout << "Index: " << index << std::endl;
  std::cout << "Stop Index: " << p.stop_index << std::endl;
  std::cout << "Time: " << hopper.t << std::endl;
  std::cout << "Contact: " << hopper.contact << std::endl;
  std::cout << "Pos Sign: " << pos_sign << std::endl;
  std::cout << "Sign Flips: " << sign_flips << std::endl;
  std::cout << "Num hops: " << hopper.num_hops << std::endl;

  // right now this is an infinite do while loop if the program state becomes stopped. 
  // TODO: add logic here to escape the loop when you want to start, will probably be commanded by
  //       an outside program
  do {
    send(*new_socket, &TX_torques, sizeof(TX_torques), 0);
    read(*new_socket, &RX_state, sizeof(RX_state));
  } while (controller.programState_ == STOPPED);
  if (index == p.stop_index || hopper.num_hops==50){
    exit(2);
  }
  index++;

  }

}
