
#include "Controller.h"

class GilManager
{
public:
   GilManager()
   {
      mThreadState = PyEval_SaveThread();
   }

   ~GilManager()
   {
      if (mThreadState)
         PyEval_RestoreThread(mThreadState);
   }

   GilManager(const GilManager&) = delete;
   GilManager& operator=(const GilManager&) = delete;
private:
   PyThreadState* mThreadState;
};

void Controller::setInitialState(vector_t initialCondition) {
  initialCondition_ = initialCondition; 
}

void Controller::resetSimulation(vector_t x0) {

  GilManager g;

  programState_ = RESET;
  t_last = -1;
  t_last_MPC = -1;

  TX_torques[0] = x0(0);
  TX_torques[1] = x0(1);
  TX_torques[2] = x0(2);
  TX_torques[3] = x0(3);
  TX_torques[4] = x0(4);
  TX_torques[5] = x0(5);
  TX_torques[6] = x0(6);
  TX_torques[7] = x0(7);
  TX_torques[8] = x0(8);
  TX_torques[9] = x0(9);
  TX_torques[10] = x0(10);
  TX_torques[11] = x0(11);
}

void Controller::stopSimulation() {
  programState_ = STOPPED;
}

void Controller::startSimulation() {
  programState_ = RUNNING;
}

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

  /* This loop will exit if the is unplugged. */
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

void setupSocket(int* new_socket, int* server_fd, struct sockaddr_in* address, uint32_t PORT) {
int opt_socket = 1;
int addrlen = sizeof(*address);
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

// get initial conditions from YAML
vector_t get_configIC() {
  
  // read the intial conditions from YAML
  YAML::Node config = YAML::LoadFile("../config/gains.yaml");
  std::vector<scalar_t> p0 = config["Simulator"]["p0"].as<std::vector<scalar_t>>();
  std::vector<scalar_t> v0 = config["Simulator"]["v0"].as<std::vector<scalar_t>>();
  std::vector<scalar_t> rpy0 = config["Simulator"]["rpy0"].as<std::vector<scalar_t>>();
  std::vector<scalar_t> w0 = config["Simulator"]["w0"].as<std::vector<scalar_t>>();

  vector_t init_cond(12);
  init_cond[0]  = p0[0];
  init_cond[1]  = p0[1];
  init_cond[2]  = p0[2];
  init_cond[3]  = v0[0];
  init_cond[4]  = v0[1];
  init_cond[5]  = v0[2];
  init_cond[6]  = rpy0[0];
  init_cond[7]  = rpy0[1];
  init_cond[8]  = rpy0[2];
  init_cond[9]  = w0[0];
  init_cond[10] = w0[1];
  init_cond[11] = w0[2];

  return init_cond;
}

// read IC from YAML ab and send to simulator
void setupSocket_sendIC(vector_t initialCondition){

  int n = initialCondition.size();
  scalar_t init_cond[n];
  for (int i=0; i<n; i++) {
        init_cond[i] = initialCondition[i];
        std::cout << initialCondition[i] << std::endl;
  }

  // regular socket stuff
  #define PORT_ 8081

  int server_IC;
  int new_socket_sim;
  struct sockaddr_in address_sender;
  int opt_sender = 1;
  int addrlen_sender = sizeof(address_sender);

  if ((server_IC = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

  if (setsockopt(server_IC, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt_sender, sizeof(opt_sender))) {
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}

  address_sender.sin_family = AF_INET;
	address_sender.sin_addr.s_addr = INADDR_ANY;
	address_sender.sin_port = htons(PORT_);

  if (bind(server_IC, (struct sockaddr*)&address_sender, sizeof(address_sender))< 0) {
		perror("bind failed");
		exit(EXIT_FAILURE);
	}

  if (listen(server_IC, 3) < 0) {
		perror("listen");
		exit(EXIT_FAILURE);
	}

  if ((new_socket_sim = accept(server_IC, (struct sockaddr*)&address_sender, (socklen_t*)&addrlen_sender)) < 0) {
		perror("accept");
		exit(EXIT_FAILURE);
	}

  send(new_socket_sim, &init_cond, sizeof(init_cond), 0);

  close(new_socket_sim);
	shutdown(server_IC, SHUT_RDWR);

}

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

Controller::Controller() {
  port.reset(new uint16_t(8080));
}

void Controller::run() {

  state.resize(20);
  q.resize(11);
  v.resize(10);
  q_local.resize(11);
  v_local.resize(10);
  q_global.resize(11);
  v_global.resize(10);
  tau.resize(10);

  int stopIndex = 0;
  GilManager g;

  ///////////// Variable Initialization ///////////////////////////
  /////////////////////////////////////////////////////////////////
  // Socket related variables
  int *new_socket = new int;
  int *server_fd = new int;
  struct sockaddr_in* address = new sockaddr_in;

  // MPC related variables
  Hopper hopper = Hopper();
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
  std::chrono::high_resolution_clock::time_point t_start;
  std::chrono::high_resolution_clock::time_point t_diff;
  
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

 //////////////////////// Main Loop /////////////////////////////////////////////////////////////////////////////////////////////////////
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // get intial conditions from YAML
    if (initialCondition_.size() == 0) {
      initialCondition_ = get_configIC();
    }
    
    setupSocket_sendIC(initialCondition_);
    
    // setup socket comms
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
    t_start = std::chrono::high_resolution_clock::now();

    programState_ == RESET; // reset on first execution

    for (;;) {
	// If the reset flag has been set, return the program state to running
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
			replan = dt_elapsed_MPC >= p.MPC_dt_replan;
			break;
			  }
		case (1): {
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

	// Simply set the next waypoint as the setpoint of the low level.
	quat_des = Quaternion<scalar_t>(x_pred(6,1), x_pred(3,1), x_pred(4,1), x_pred(5,1));
	omega_des << x_pred(14,1), x_pred(15,1),x_pred(16,1);
	u_des = u_pred;

	quat_term = Quaternion<scalar_t>(x_term(6), x_term(3), x_term(4), x_term(5));
	pos_term << x_term(0), x_term(1), x_term(2);

	if (dt_elapsed > p.dt) {
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


	// if (stopIndex ==0) {
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

	TX_torques[25] = (scalar_t) programState_;
	// } else {
	// stopIndex = 0;
	// programState_ = RESET;
	// }

	// reset MPC
	// NOTICE: time gets reset, so this loops. Do not use state(0) [time] as a flag,
	// becuase it will be reset by the simulator
 // if (state(0) > 3.) {
	  //startSimulation();
	  //stopSimulation();
//	  vector_t x0(12);
//	  x0 << 0,0,15,0,0,0,0,0,0,0,0,0;
//	  resetSimulation(x0);
//  }


  // count number of hops based on changing directions of z velocity
  pos_sign_check = hopper.vel(2) > 0;
  if (pos_sign != pos_sign_check) {
    pos_sign = pos_sign_check;
    sign_flips ++;
    if (sign_flips % 2 ==0)
      hopper.num_hops++;
  }

  //std::cout << "Index: " << index << std::endl;
  //std::cout << "Stop Index: " << p.stop_index << std::endl;
  //std::cout << "Time: " << hopper.t << std::endl;
  //std::cout << "Contact: " << hopper.contact << std::endl;
  //std::cout << "Pos Sign: " << pos_sign << std::endl;
  //std::cout << "Sign Flips: " << sign_flips << std::endl;
  //std::cout << "Num hops: " << hopper.num_hops << std::endl;

  // right now this is an infinite do while loop if the program state becomes stopped. 
  // TODO: add logic here to escape the loop when you want to start, will probably be commanded by
  //       an outside program
  do {
    send(*new_socket, &TX_torques, sizeof(TX_torques), 0);
    read(*new_socket, &RX_state, sizeof(RX_state));
    stopIndex++;
  } while (programState_ == STOPPED);
  stopIndex--;
  if (index == p.stop_index || hopper.num_hops==50){
    return;
  }
  if (duration > 0 && (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count()) > duration) {
	  return;
  }
  // if (programState_ == RESET) {
  //   programState_ = RUNNING;
  // }
  index++;

  }


}
