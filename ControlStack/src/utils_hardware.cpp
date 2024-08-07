#include <string>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/String.h"

#include "../inc/Types.h"
#include "../inc/utils.h"
#include <condition_variable>

char send_buff[40];
float desstate[10];
char buff[52];
float states[13];
vector_t ESPstate(13);
volatile bool ESP_initialized = false;
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

struct HardwareParameters
{
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
  std::string model_name;
  scalar_t v_max;
  scalar_t a_max;
  scalar_t dt_replan;
  int optiTrackSetting;
  std::string rom_type;
  int horizon;
} p;

std::mutex state_mtx;
std::mutex des_state_mtx;
int sock;
sockaddr_in source, destination;
sockaddr_in senderAddr;
socklen_t destinationAddrLen;
socklen_t senderAddrLen;

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

void setupGainsHardware(const std::string filepath) 
{
    YAML::Node config = YAML::LoadFile(filepath);
    p.orientation_kp = config["Orientation"]["Kp"].as<std::vector<scalar_t>>();
    p.orientation_kd = config["Orientation"]["Kd"].as<std::vector<scalar_t>>();
    p.leg_kp = config["Leg"]["Kp"].as<scalar_t>();
    p.leg_kd = config["Leg"]["Kd"].as<scalar_t>();
    p.dt = config["Debug"]["dt"].as<scalar_t>();
    p.frameOffset = config["MPC"]["frameOffset"].as<scalar_t>();
    p.markerOffset = config["MPC"]["markerOffset"].as<scalar_t>();
    p.predHorizon = config["Debug"]["predHorizon"].as<int>();
    p.stop_index = config["Debug"]["stopIndex"].as<int>();
    p.p0 = config["Simulator"]["p0"].as<std::vector<scalar_t>>();
    p.gains.resize(8);
    p.gains << p.orientation_kp[0], p.orientation_kp[1], p.orientation_kp[2],
            p.orientation_kd[0], p.orientation_kd[1], p.orientation_kd[2],
            p.leg_kp, p.leg_kd;
            
    p.model_name = config["RL"]["model_name"].as<std::string>();
    p.rom_type = config["RL"]["rom_type"].as<std::string>();
    p.v_max = config["RL"]["v_max"].as<scalar_t>();
    p.a_max = config["RL"]["a_max"].as<scalar_t>();
    p.dt_replan = config["RL"]["dt_replan"].as<scalar_t>();
    p.horizon = config["RL"]["horizon"].as<scalar_t>();
}


void getStateFromEthernet(scalar_t &reset, std::condition_variable &cv, std::mutex &m)
{

  ssize_t recvBytes = 0;
  ssize_t n_bytes = 0;

  while (1)
  {

    // encode send_buff
    {
      std::lock_guard<std::mutex> lck(des_state_mtx);
      memcpy(send_buff, desstate, 10 * 4);
    }

    // std::cout<<"Writing..."<<std::endl;
    if (reset > 0.5)
    {
      send_buff[0] = 1;
      send_buff[1] = 2;
      send_buff[2] = 3;
      send_buff[3] = 4;
      send_buff[4] = 5;
      send_buff[5] = 6;
      send_buff[6] = 7;
      send_buff[7] = 8;
      reset = 0;
    }

    n_bytes = ::sendto(sock, send_buff, sizeof(send_buff), 0, reinterpret_cast<sockaddr *>(&destination), destinationAddrLen);

    // receive string states, ESP8266 -> PC
    //  std::cout<<"Reading..."<<std::endl;
    recvBytes = ::recvfrom(sock, buff, sizeof(buff), 0, reinterpret_cast<sockaddr *>(&senderAddr), &senderAddrLen);

    float data[13] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::memcpy(data, buff, sizeof(data));

    memcpy(&states, buff, 13 * sizeof(float));
    quat_t quat_a = quat_t(states[6], states[7], states[8], states[9]);
    quat_a.normalize();
    {
      std::lock_guard<std::mutex> lck(state_mtx);
      ESPstate << states[0], states[1], states[2], states[3], states[4], states[5], quat_a.w(), quat_a.x(), quat_a.y(), quat_a.z(), states[10], states[11], states[12];
      ESP_initialized = true;
    }
  }

  ::close(sock);
}

void signal_callback_handler(int signum) {
        std::cout << "Caught signal " << signum << std::endl;
   // Terminate program
   exit(signum);
}

void setupSocketHardware()
{

  std::string hostname{"10.0.0.6"}; // IP Adress of the MACHINE communicating with the teensy
  std::string destname{"10.0.0.7"}; // IP Address of the TEENSY communicating with the machine
  uint16_t port = 4333;             // Port over which you want to communicate

  sock = ::socket(AF_INET, SOCK_DGRAM, 0); // AF_INET for IPV4, SOCK_DGRAM for UDP, 0 for IP (protocol value)

  if (sock == -1)
  {
    printf("socket creation failed...\n");
    exit(0);
  }
  else
  {
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

  int val = bind(sock, (struct sockaddr *)&source,
                 sizeof(source));

  senderAddr.sin_family = AF_INET;
  senderAddr.sin_port = htons(port);
  senderAddr.sin_addr.s_addr = inet_addr(hostname.c_str());
  senderAddrLen = sizeof(senderAddr);
  sleep(1);
}