#include "../inc/utils.h"

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

void hi(int num) {
    std::cout << "hi: " << num <<  std::endl;
}

void setupSocket(int &server_fd, int &new_socket, struct sockaddr_in &address, int opt_socket, int &addrlen) {
    // server_fd = new int;
    // new_socket = new int;
    // address = new sockaddr_in;

    addrlen = sizeof(address);

// Socket stuff
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                   &opt_socket, sizeof(opt_socket))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr*) &address,
             sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr*) &address,
                              (socklen_t *) &addrlen)) < 0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }
}

void setupGains(const std::string filepath, MPC_Parameters &mpc_p, Parameters &p) {
    // Read gain yaml
    YAML::Node config = YAML::LoadFile(filepath);
    p.dt = config["LowLevel"]["dt"].as<scalar_t>();

    p.roll_offset = config["roll_offset"].as<scalar_t>();
    p.pitch_offset = config["pitch_offset"].as<scalar_t>();
    p.yaw_drift = config["Simulator"]["yaw_drift"].as<scalar_t>();
    std::vector<scalar_t> tmp = config["Simulator"]["p0"].as<std::vector<scalar_t>>();
    p.x0 = tmp[0];
    p.y0 = tmp[1];
    p.rom_type = "pred_cbf";
    
    p.horizon = config["PredCBF"]["horizon"].as<scalar_t>();
    p.alpha = config["PredCBF"]["alpha"].as<scalar_t>();
    p.rho = config["PredCBF"]["rho"].as<scalar_t>();
    p.smooth_barrier = config["PredCBF"]["smooth_barrier"].as<bool>();
    p.epsilon = config["PredCBF"]["epsilon"].as<scalar_t>(); 
    p.k_r = config["PredCBF"]["k_r"].as<scalar_t>();   
    p.v_max = config["PredCBF"]["v_max"].as<scalar_t>();
    p.pred_dt = config["PredCBF"]["pred_dt"].as<scalar_t>();
    p.dt_replan = config["PredCBF"]["dt"].as<scalar_t>();
    p.iters = config["PredCBF"]["iters"].as<int>();
    p.K = config["PredCBF"]["K"].as<scalar_t>();
    p.tol = config["PredCBF"]["tol"].as<scalar_t>();
    p.use_delta = config["PredCBF"]["use_delta"].as<bool>();
    p.rs = config["PredCBF"]["rs"].as<std::vector<scalar_t>>();
    p.cxs = config["PredCBF"]["cxs"].as<std::vector<scalar_t>>();
    p.cys = config["PredCBF"]["cys"].as<std::vector<scalar_t>>();
    std::vector<scalar_t> zd = config["PredCBF"]["zd"].as<std::vector<scalar_t>>();
    p.zd << zd[0], zd[1];
    p.dt_planner = config["dt_planner"].as<scalar_t>();

    // mpc_p.N = config["MPC"]["N"].as<int>();
    // mpc_p.SQP_iter = config["MPC"]["SQP_iter"].as<int>();
    // mpc_p.discountFactor = config["MPC"]["discountFactor"].as<scalar_t>();
    // tmp = config["MPC"]["stateScaling"].as<std::vector<scalar_t>>();
    // mpc_p.dt_flight= config["MPC"]["dt_flight"].as<scalar_t>();
    // mpc_p.dt_ground = config["MPC"]["dt_ground"].as<scalar_t>();
    // mpc_p.MPC_dt_replan = config["MPC"]["dt_replan"].as<scalar_t>();
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
    // mpc_p.max_vel = config["MPC"]["max_vel"].as<scalar_t>();
}

vector_3t Quaternion2Euler(const quat_t &q)
{
    vector_3t euler;
    euler[0] = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y())); // Roll
    euler[1] = asin(2 * (q.w() * q.y() - q.z() * q.x()));                                           // Pitch
    euler[2] = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z())); // Yaw
    return euler;
}

quat_t minus(quat_t q_1, quat_t q_2) { return q_2.inverse() * q_1; }
quat_t plus(quat_t q_1, quat_t q_2) { return q_1 * q_2; }
scalar_t extract_yaw(quat_t q) { return atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (pow(q.y(), 2) + pow(q.z(), 2))); }


matrix_3t cross(vector_3t q)
{
    matrix_3t c;
    c << 0, -q(2), q(1),
        q(2), 0, -q(0),
        -q(1), q(0), 0;
    return c;
}

matrix_3t quat2Rot(quat_t q)
{
    scalar_t qw, qx, qy, qz;
    matrix_3t Rq;
    qw = q.w();
    qx = q.x();
    qy = q.y();
    qz = q.z();
    Rq << pow(qw, 2) + pow(qx, 2) - pow(qy, 2) - pow(qz, 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy),
        2 * (qx * qy + qw * qz), pow(qw, 2) - pow(qx, 2) + pow(qy, 2) - pow(qz, 2), 2 * (qy * qz - qw * qx),
        2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), pow(qw, 2) - pow(qx, 2) - pow(qy, 2) + pow(qz, 2);
    return Rq;
}

vector_t Log(vector_t x) {
  vector_t g_frak(20);
  quat_t quat(x(6), x(3), x(4), x(5));
  auto quat_ = manif::SO3<scalar_t>(quat);
  manif::SO3Tangent<scalar_t> xi = quat_.log();
  g_frak << x.segment(0,3),xi.coeffs(),x.segment(7,4),x.segment(11,10);
  return g_frak;
}

vector_t Exp(vector_t xi) {
  vector_t g(21);
  manif::SO3Tangent<scalar_t> xi_;
  xi_ << xi(3),xi(4),xi(5);
  quat_t quat = xi_.exp().quat();
  g << xi.segment(0,3), quat.coeffs(), xi.segment(6,14);
  return g;
}

vector_t qk_to_xik(vector_t qk, vector_t q0) {
  quat_t quat0(q0(6), q0(3), q0(4), q0(5));
  quat_t quatk(qk(6), qk(3), qk(4), qk(5));

  vector_t tmp(21);
  tmp << qk.segment(0,3), (quat0.inverse()*quatk).coeffs(), qk.segment(7,14);
  vector_t xik(20);
  xik = Log(tmp);
  return xik;
}

vector_t xik_to_qk(vector_t xik, vector_t q0) {
  vector_t tmp(21);
  tmp = Exp(xik);
  quat_t quat0(q0(6), q0(3), q0(4), q0(5));
  quat_t quatk(tmp(6), tmp(3), tmp(4), tmp(5));
  vector_t qk(21);
  qk << tmp.segment(0,3), (quat0*quatk).coeffs(), tmp.segment(7,14);
  //qk << tmp.segment(0,3), (quatk).coeffs(), tmp.segment(7,14);
  return qk;
}

vector_t global2local(vector_t x_g) {
	vector_t q(11);
	vector_t v(10);
	vector_t q_local(11);
	vector_t v_local(10);
	vector_t x_l(21);

	q << x_g.head(11);
	v << x_g.tail(10);
        quat_t quat(q(6), q(3), q(4), q(5));
        auto quat_ = manif::SO3<scalar_t>(quat);
        matrix_3t Rq = quat2Rot(quat);
        q_local << quat.inverse()._transformVector(q.segment(0,3)), quat.coeffs(),q.segment(7,4);
        v_local << quat.inverse()._transformVector(v.segment(0,3)), quat.inverse()._transformVector(v.segment(3,3)),v.segment(6,4);
	// Both of these below formulations are wrong but are left as posterity
	// Murray Notes
	//v_local << quat.inverse()._transformVector(v.segment(0,3)) - quat.inverse()._transformVector(Hopper::cross(q.segment(0,3))*v.segment(3,3)), quat.inverse()._transformVector(v.segment(3,3)),v.segment(6,4);
	// Hacky right trivialization instead of left, needed to transform the omega instead
        //v_local << quat.inverse()._transformVector(v.segment(0,3)) + quat.inverse()._transformVector(Hopper::cross(q_local.segment(0,3))*v.segment(3,3)), v.segment(3,7);
	x_l << q_local, v_local;
        return x_l;
}

vector_t local2global(vector_t x_l) {
	vector_t q(11);
	vector_t v(10);
	vector_t q_global(11);
	vector_t v_global(10);
	vector_t x_g(21);

	q << x_l.head(11);
	v << x_l.tail(10);
	vector_3t w = v.segment(3,3);
	vector_3t p = q.segment(0,3);
        quat_t quat(q(6), q(3), q(4), q(5));
        matrix_3t Rq = quat2Rot(quat);
        q_global << quat._transformVector(q.segment(0,3)), quat.coeffs(), q.segment(7,4);
        v_global << quat._transformVector(v.segment(0,3)), quat._transformVector(v.segment(3,3)),v.segment(6,4);
	// Both of these below formulations are wrong but are left as posterity
        // Murray notes:
	//v_global << quat._transformVector(v.segment(0,3)) + Hopper::cross(q_global.segment(0,3))*quat._transformVector(w), quat._transformVector(w),v.segment(6,4);
	// Hacky right trivialization instead of left, needed to transform the omega instead
        //v_global << quat._transformVector(v.segment(0,3)) - quat._transformVector(Hopper::cross(p)*w), quat._transformVector(w), v.segment(6,4);
	x_g << q_global, v_global;
        return x_g;
}