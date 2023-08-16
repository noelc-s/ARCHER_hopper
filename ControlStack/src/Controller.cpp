// for joystick inputs
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>

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
//#include "pinocchio/algorithm/kinematics.hpp"

#define PORT 8080
#define MAXLINE 1000

using namespace Eigen;
using namespace Hopper_t;
using namespace pinocchio;

const static IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

///////////////////////////////////////////////////////////////////////////////////////////

// command is the floating object that needs to be altered within the function
// Need to append here to get PS4 inputs
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

///////////////////////////////////////////////////////////////////////////////////////////

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

void getJoystickInput(vector_3t &command, vector_2t &dist, std::condition_variable & cv, std::mutex & m)
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

  //scaling factor (joysticks vals in [-32767 , +32767], signed 16-bit)
  double comm_scale = 10000.;
  double dist_scale = 7000.;

  /* This loop will exit if the controller is unplugged. */
  while (read_event(js, &event) == 0)
  {
    switch(event.type) {
      
      // moving a joystick
      case JS_EVENT_AXIS:
        axis = get_axis_state(&event, axes);
        if (axis == 0) { 
          command << axes[axis].x / comm_scale, -axes[axis].y / comm_scale, 0; // Left Joy Stick
          std::cout << "Command: " << command[0] << ", " << command[1] << std::endl;
        }
        if (axis == 1) {
          dist << axes[axis].x / dist_scale, -axes[axis].y / dist_scale; // Right Joy Stick
          std::cout << "Disturbance: " << dist[0] << ", " << dist[1] << std::endl;
        }
        break;

      // pressed a button
      case JS_EVENT_BUTTON:
        if (event.number == 0 && event.value == 1){ //pressed 'X'
          command << 0,0,1;
          std::cout << "Flip: " << std::endl;
          }
        break;
      
      // ignore init events
      default:
        break;
    }
  }

  close(js);
}

///////////////////////////////////////////////////////////////////////////////////////////

int *server_fd = new int;
int *new_socket = new int;
int valread;
struct sockaddr_in *address = new sockaddr_in;
int opt_socket = 1;
int addrlen = sizeof(*address);
scalar_t TX_torques[13+2*5+2] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
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
    scalar_t dt;
    scalar_t MPC_dt_flight;
    scalar_t MPC_dt_ground;
    scalar_t MPC_dt_replan;
    int stop_index; 
} p;

void setupGains(const std::string filepath, MPC::MPC_Params &mpc_p) {
    // Read gain yaml
    YAML::Node config = YAML::LoadFile(filepath);
    p.dt = config["LowLevel"]["dt"].as<scalar_t>();
    p.MPC_dt_ground = config["MPC"]["dt_ground"].as<scalar_t>();
    p.MPC_dt_flight = config["MPC"]["dt_flight"].as<scalar_t>();
    p.MPC_dt_replan = config["MPC"]["dt_replan"].as<scalar_t>();
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
  
    p.stop_index = 100000;
    
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

///////////////////////////////////////////////////////////////////////////////////////////

// Driver code
int main() {

  //***************************************************
  
  // // trajectory test
  // vector_array_t waypts;
  // scalar_array_t times;
 
  // vector_t vec(12);
  // vec.setZero();

  // vec.segment(0,2) << 0,0;
  // waypts.push_back(vec);
  // vec.segment(0,2) << -1,0;
  // waypts.push_back(vec);
  // vec.segment(0,2) << -1,1;
  // waypts.push_back(vec);
  // vec.segment(0,2) << 0,0;
  // waypts.push_back(vec);
  // vec.segment(0,2) << 1.9,0;
  // waypts.push_back(vec);
  // vec.segment(0,2) << 3.5,0;
  // waypts.push_back(vec);

  // for (int i=0; i<waypts.size(); i++) {
  //   times.push_back(10.* (float) i);
  // }

  // Traj trajectory = {waypts,times, waypts.size()};

  // Bezier_T tra(trajectory, 1.0);

  ////////////////////////// TRAJECTORY //////////////////
    // follow trajecotry test
    vector_array_t waypts;
    scalar_array_t times;

    vector_t tmp(12);
    tmp.setZero();
  
    tmp.segment(0,2) << 0,0;
    waypts.push_back(tmp);
    tmp.segment(0,2) << 0.01, 0.01;
    waypts.push_back(tmp);

    for (int i=0; i<waypts.size(); i++) {
      times.push_back(15.* (float) i);
    }
    
    Traj trajectory = {waypts, times, waypts.size()};
    Bezier_T tra(trajectory, 1.0);

    /////////////////////////////////////////////////////////

  //***************************************************
  
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

  //***************************************************

    setupSocket();    
    MPC::MPC_Params mpc_p;
    setupGains("../config/gains.yaml", mpc_p);

    // Read yaml

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
    bool fileWrite = true;
    time_t now = time(0);  // current date/time based on current system
    tm *ltm = localtime(&now);
    std::string tot_time = std::to_string(now);
    std::string yr = std::to_string(1900+ltm->tm_year);
    std::string mn = std::to_string(1+ltm->tm_mon);
    std::string dy = std::to_string(ltm->tm_mday);
    std::string tm = std::to_string(ltm->tm_hour) + ":" +
                     std::to_string(ltm->tm_min) + ":" + 
                     std::to_string(ltm->tm_sec); 
    std::string time_now = tot_time + "_" + yr + "_" + mn + "_" + dy + "_" + tm;

    std::string dataLog = "../data/data_sim/" + time_now + ".csv";
    std::ofstream fileHandle;
    fileHandle.open(dataLog);
    fileHandle << "t,x,y,z,q_w,q_x,q_y,q_z,x_dot,y_dot,z_dot,w_1,w_2,w_3,contact,l,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,z_acc";

    int index = 1;
    
    // for counting number of hops based on z velocity
    hopper.num_hops = 0;
    int sign_flips =0;
    bool pos_sign = hopper.vel(2) > 0;
    bool pos_sign_check;

    scalar_t t_last = -1;
    scalar_t dt_elapsed;
    scalar_t t_last_MPC = -1;
    scalar_t dt_elapsed_MPC;

    quat_t quat_des = Quaternion<scalar_t>(1,0,0,0);
    vector_3t omega_des;
    omega_des.setZero();
    vector_t u_des(4);
    u_des.setZero();

    // MPC object
    MPC opt = MPC(20, 4, mpc_p);
    vector_t sol(opt.nx*opt.p.N+opt.nu*(opt.p.N-1));
    vector_t sol_g((opt.nx+1)*opt.p.N+opt.nu*(opt.p.N-1));
    sol.setZero();
    sol_g.setZero();

    std::condition_variable cv;
    std::mutex m;
    vector_3t command;
    vector_2t dist;
    vector_2t command_interp;
    std::thread userInput(getJoystickInput, std::ref(command), std::ref(dist), std::ref(cv), std::ref(m));
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
	hopper.v.segment(3,3) = quat._transformVector(hopper.v.segment(3,3)); // Turn local omega to global omega
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
          opt.solve(hopper, sol, command, command_interp, &tra); ///////////////////////////////////
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

	// Compute continuous time solution to discrete time problem. 
        //vector_t x_des(21);
	//hopper.css2dss(opt.Ac.block(0,0,opt.nx,opt.nx),opt.Bc.block(0,0,opt.nx,opt.nu),opt.Cc.block(0,0,opt.nx,1),state(0)-t_last_MPC,opt.Ad_,opt.Bd_,opt.Cd_);
	//x_des << MPC::local2global(MPC::Exp(opt.Ad_*sol.segment(0,20) + opt.Bd_*u_pred + opt.Cd_));
	//quat_des = Quaternion<scalar_t>(x_des(6), x_des(3), x_des(4), x_des(5));
	//omega_des << x_des(14), x_des(15),x_des(16);
	
	// Simply set the next waypoint as the setpoint of the low level.
	quat_des = Quaternion<scalar_t>(x_pred(6,1), x_pred(3,1), x_pred(4,1), x_pred(5,1));
	omega_des << x_pred(14,1), x_pred(15,1),x_pred(16,1);
	u_des = u_pred;

	quat_term = Quaternion<scalar_t>(x_term(6), x_term(3), x_term(4), x_term(5));
	pos_term << x_term(0), x_term(1), x_term(2);

	if (dt_elapsed > p.dt) {
		// Uncomment below to isolate the low level controller for testing
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

	// TODO: Right now, the impact map takes up a dt, but that makes the timing inconsistent. Fix that.
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


  // count number of hops based on changing directions of z velocity
  pos_sign_check = hopper.vel(2) > 0;
  if (pos_sign != pos_sign_check) {
    pos_sign = pos_sign_check;
    sign_flips ++;
    if (sign_flips % 2 ==0)
      hopper.num_hops++;
  }

  
 // std::cout << "Index: " << index << std::endl;
 // std::cout << "Stop Index: " << p.stop_index << std::endl;
 // std::cout << "Time: " << hopper.t << std::endl;
 // std::cout << "Contact: " << hopper.contact << std::endl;
 // std::cout << "Pos Sign: " << pos_sign << std::endl;
 // std::cout << "Sign Flips: " << sign_flips << std::endl;
 // std::cout << "Num hops: " << hopper.num_hops << std::endl;


  send(*new_socket, &TX_torques, sizeof(TX_torques), 0);
	if (index == p.stop_index || hopper.num_hops==500){
		exit(2);
	}
	index++;
  }

}
