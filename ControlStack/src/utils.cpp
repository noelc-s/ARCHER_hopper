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

void setupTubeSocket(int &server_fd, int &new_socket, struct sockaddr_in &address, int opt_socket, int &addrlen) {
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
    address.sin_port = htons(TUBE_PORT);

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

void setupGains(const std::string filepath, Parameters &p) {
    // Read gain yaml
    YAML::Node config = YAML::LoadFile(filepath);
    p.dt = config["LowLevel"]["dt"].as<scalar_t>();

    p.roll_offset = config["roll_offset"].as<scalar_t>();
    p.pitch_offset = config["pitch_offset"].as<scalar_t>();
    p.yaw_drift = config["Simulator"]["yaw_drift"].as<scalar_t>();
    p.rom_type = config["MPC"]["rom"]["cls"].as<std::string>();
    p.v_max = config["MPC"]["rom"]["v_max"].as<scalar_t>();
    p.a_max = config["MPC"]["rom"]["a_max"].as<scalar_t>();
    p.dt_replan = config["MPC"]["rom"]["dt"].as<scalar_t>();
    p.horizon = config["MPC"]["N"].as<scalar_t>();

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
