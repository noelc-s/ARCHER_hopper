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

#include <librealsense2/rs.hpp>

char send_buff[40];
float desstate[10];
char buff[52];
float states[13];
vector_t ESPstate(13);
volatile bool ESP_initialized = false;
struct State
{
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
  scalar_t dt_lowlevel;
  scalar_t dt_policy;
  int optiTrackSetting;
  std::string rom_type;
  int horizon;
  scalar_t zed_x_offset;
  scalar_t zed_y_offset;
} p;

std::mutex state_mtx;
std::mutex des_state_mtx;
int sock;
sockaddr_in source_sock, destination;
sockaddr_in senderAddr;
socklen_t destinationAddrLen;
socklen_t senderAddrLen;
scalar_t alpha; // filtering alpha

// Kalman Filter gains
matrix_t A_kf(6, 6);
matrix_t B_kf(6, 4);
bool contact = false;

void MujocoVis(std::condition_variable &cv, Hopper::State &state, float* TX_torques,  scalar_t *RX_state, int size) {
  // Socket Stuff for Mujoco vis
  int server_fd;
  int new_socket;
  // int valread;
  struct sockaddr_in address;
  int opt_socket = 1;
  int addrlen;

  std::cout << "Waiting for MuJoCo visualizer" << std::endl;
  setupSocket(server_fd, new_socket, address, opt_socket, addrlen);

  while(1) {
    auto ret = read(new_socket, RX_state, sizeof(RX_state));  // Correct size for RX_state (static array)
    send(new_socket, TX_torques, size * sizeof(float), 0);  // Send the entire TX_torques array
  }

}

// Find devices with specified streams
bool device_with_streams(std::vector <rs2_stream> stream_requests, std::string& out_serial)
{
    rs2::context ctx;
    auto devs = ctx.query_devices();
    std::vector <rs2_stream> unavailable_streams = stream_requests;
    for (auto dev : devs)
    {
        std::map<rs2_stream, bool> found_streams;
        for (auto& type : stream_requests)
        {
            found_streams[type] = false;
            for (auto& sensor : dev.query_sensors())
            {
                for (auto& profile : sensor.get_stream_profiles())
                {
                    if (profile.stream_type() == type)
                    {
                        found_streams[type] = true;
                        unavailable_streams.erase(std::remove(unavailable_streams.begin(), unavailable_streams.end(), type), unavailable_streams.end());
                        if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
                            out_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                    }
                }
            }
        }
        // Check if all streams are found in current device
        bool found_all_streams = true;
        for (auto& stream : found_streams)
        {
            if (!stream.second)
            {
                found_all_streams = false;
                break;
            }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
        }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
        if (found_all_streams)                                                                                                                                                                                                                                                                                                                                                                                                                                                                
            return true;                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
    }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
    // After scanning all devices, not all requested streams were found                                                                                                                                                                                                                                                                                                                                                                                                                       
    for (auto& type : unavailable_streams)                                                                                                                                                                                                                                                                                                                                                                                                                                                    
    {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
        switch (type)                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
        {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
        case RS2_STREAM_POSE:                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
        case RS2_STREAM_FISHEYE:                                                                                                                                                                                                                                                                                                                                                                                                                                                              
            std::cerr << "Connect T26X and rerun the demo" << std::endl;                                                                                                                                                                                                                                                                                                                                                                                                                      
            break;                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        case RS2_STREAM_DEPTH:                                                                                                                                                                                                                                                                                                                                                                                                                                                                
            std::cerr << "The demo requires Realsense camera with DEPTH sensor" << std::endl;                                                                                                                                                                                                                                                                                                                                                                                                 
            break;                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        case RS2_STREAM_COLOR:                                                                                                                                                                                                                                                                                                                                                                                                                                                                
            std::cerr << "The demo requires Realsense camera with RGB sensor" << std::endl;                                                                                                                                                                                                                                                                                                                                                                                                   
            break;                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        default:                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
            throw std::runtime_error("The requested stream: " + std::to_string(type) + ", for the demo is not supported by connected devices!"); // stream type                                                                                                                                                                                                                                                                                                                               
        }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
    return false;                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
}

inline rs2_quaternion quaternion_exp(rs2_vector v)
{
    float x = v.x/2, y = v.y/2, z = v.z/2, th2, th = sqrtf(th2 = x*x + y*y + z*z);
    float c = cosf(th), s = th2 < sqrtf(120*FLT_EPSILON) ? 1-th2/6 : sinf(th)/th;
    rs2_quaternion Q = { s*x, s*y, s*z, c };
    return Q;
}

inline rs2_quaternion quaternion_multiply(rs2_quaternion a, rs2_quaternion b)
{
    rs2_quaternion Q = {
        a.x * b.w + a.w * b.x - a.z * b.y + a.y * b.z,
        a.y * b.w + a.z * b.x + a.w * b.y - a.x * b.z,
        a.z * b.w - a.y * b.x + a.x * b.y + a.w * b.z,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
    return Q;
}


rs2_pose predict_pose(rs2_pose & pose, float dt_s)
{
    rs2_pose P = pose;
    P.translation.x = dt_s * (dt_s/2 * pose.acceleration.x + pose.velocity.x) + pose.translation.x;
    P.translation.y = dt_s * (dt_s/2 * pose.acceleration.y + pose.velocity.y) + pose.translation.y;
    P.translation.z = dt_s * (dt_s/2 * pose.acceleration.z + pose.velocity.z) + pose.translation.z;
    rs2_vector W = {
            dt_s * (dt_s/2 * pose.angular_acceleration.x + pose.angular_velocity.x),
            dt_s * (dt_s/2 * pose.angular_acceleration.y + pose.angular_velocity.y),
            dt_s * (dt_s/2 * pose.angular_acceleration.z + pose.angular_velocity.z),
    };
    P.rotation = quaternion_multiply(quaternion_exp(W), pose.rotation);
    return P;
}


  Eigen::Quaternion<double> q;  
  Eigen::Matrix<double, 3, 1> v;
  Eigen::Matrix<double, 3, 3> R_RS_to_RS_aligned;
  Eigen::Matrix<double, 3, 3> R_RS_aligned_to_H;
  Eigen::Matrix<double, 3, 3> R;                



vector_3t realsense_pos{0,0,0};
vector_3t realsense_vel{0,0,0};
quat_t realsense_q;

void realSenseLoop(scalar_t& yaw) {
std::string serial;
    if (!device_with_streams({ RS2_STREAM_POSE }, serial))
        return;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    if (!serial.empty())
        cfg.enable_device(serial);
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    auto callback = [&](const rs2::frame& frame)
    {
        if (rs2::pose_frame fp = frame.as<rs2::pose_frame>()) {
            rs2_pose pose_data = fp.get_pose_data();
            auto now = std::chrono::system_clock::now().time_since_epoch();
            double now_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
            double pose_time_ms = fp.get_timestamp();
            float dt_s = static_cast<float>(std::max(0., (now_ms - pose_time_ms)/1000.));
            rs2_pose predicted_pose = predict_pose(pose_data, dt_s);
             
            realsense_pos << predicted_pose.translation.z,
                          predicted_pose.translation.x,
                          predicted_pose.translation.y; //  6 4 5
            matrix_3t R_yaw;
            R_yaw << cos(yaw),-sin(yaw),0,
              sin(yaw),cos(yaw),0,
              0,0,1;
            realsense_pos = R_yaw * realsense_pos;

	    realsense_vel << predicted_pose.velocity.x,
			  predicted_pose.velocity.y,
			  predicted_pose.velocity.z;
            q = Eigen::Quaternion<double>(pose_data.rotation.w, pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.z);
            realsense_vel = R.transpose()*(q.inverse() * realsense_vel);
        }
    };

    rs2::pipeline_profile profiles = pipe.start(cfg, callback);
    std::cout << "started RealSense\n";
    while(true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  static std::chrono::high_resolution_clock::time_point t1;
  t1 = std::chrono::high_resolution_clock::now();
  static std::chrono::seconds oneSecond(1);
  static std::chrono::high_resolution_clock::time_point last_t_state_log = t1 - oneSecond;
  static bool init = false;
  static scalar_t dt;
  static vector_3t state_init(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z + p.frameOffset + p.markerOffset);
  static vector_3t current_vel(0, 0, 0);
  static vector_3t filtered_current_vel(0, 0, 0);
  static bool first_contact = false;

  static vector_3t previous_vel(0, 0, 0);
  static vector_3t last_state(0.,0., msg->pose.position.z + p.frameOffset + p.markerOffset);
  static vector_6t est_state(6);

  if (!init) 
  {
    est_state.setZero();
    est_state.segment(0,3) << last_state;
    R_RS_to_RS_aligned << 0.271397251061513,0.863510292339979,0.425080588993638,
                0.962467418729722,-0.243493249790936,-0.119864528489434,
                1.01307850997046E-15,0.441657120772634,-0.897183920760302;  
    R_RS_aligned_to_H << 0,1,0,
                        0, 0, 1,
                        1, 0, 0;
    R = R_RS_to_RS_aligned * R_RS_aligned_to_H;
    init = true;
  }


  static const scalar_t g = 9.81;

  // else
  // local
  // OptiState.x = msg->pose.position.x - state_init(0);
  // OptiState.y = msg->pose.position.y - state_init(1);
  // global
  // OptiState.x = msg->pose.position.x;
  // OptiState.y = msg->pose.position.y;
  // OptiState.z = msg->pose.position.z + p.frameOffset + p.markerOffset;

  OptiState.x = realsense_pos(0);
  OptiState.y = realsense_pos(1);
  OptiState.z = realsense_pos(2);

  // std::cout << msg->pose.position.x << ", " << msg->pose.position.y << ", " << msg->pose.position.z
  //          << ", " << realsense_state(0)<< ", " << realsense_state(1)<< ", " << realsense_state(2) << std::endl;

  OptiState.q_w = msg->pose.orientation.w;
  OptiState.q_x = msg->pose.orientation.x;
  OptiState.q_y = msg->pose.orientation.y;
  OptiState.q_z = msg->pose.orientation.z;

  dt = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - last_t_state_log).count() * 1e-9;
  if (dt < 1e-3)
    return;

  current_vel << (OptiState.x - last_state(0)) / dt, (OptiState.y - last_state(1)) / dt, (OptiState.z - last_state(2)) / dt;
  filtered_current_vel << alpha * current_vel + (1 - alpha) * previous_vel;
  last_state << OptiState.x, OptiState.y, OptiState.z;

  if (contact)
    first_contact = true;

  // Kalman Filter
  vector_4t input(-g, OptiState.x, OptiState.y, OptiState.z);
  est_state << A_kf * est_state + B_kf * input;
  if (!first_contact || contact)
  {
    est_state(2) = OptiState.z;
    est_state(5) = filtered_current_vel(2);
  }

  OptiState.x = realsense_pos(0);
  OptiState.y = realsense_pos(1);
  OptiState.z = realsense_pos(2);
  OptiState.x_dot = realsense_vel(0);
  OptiState.y_dot = realsense_vel(1);
  OptiState.z_dot = realsense_vel(2);

  //OptiState.x = est_state(0);
  //OptiState.y = est_state(1);
  //OptiState.z = est_state(2);
  //OptiState.x_dot = est_state(3);
  //OptiState.y_dot = est_state(4);
  //OptiState.z_dot = est_state(5);

  // for ii = 2:size(xyz, 1)

  //         % predict
  //         xyz_hat = A * estim(ii-1, :)' + B * -u;
  //         % update
  //         estim(ii, :) = xyz_hat + K * (xyz(ii, :)' - C * xyz_hat);
  //         estim2(ii, :) = kf.A * estim2(ii-1, :)' + kf.B * [-u; xyz(ii, :)'];
  //     if contact_sampled(ii)
  //         estim(ii, [3 6]) = [xyz(ii, 3) finite_diff_vel(ii,3)];
  //         estim2(ii, [3 6]) = [xyz(ii, 3) finite_diff_vel(ii,3)];
  //     end
  // end

  // OptiState.x_dot = alpha * current_vel(0) + (1-alpha) * previous_vel(0);
  // OptiState.y_dot = alpha * current_vel(1) + (1-alpha) * previous_vel(1);
  // OptiState.z_dot = alpha * current_vel(2) + (1-alpha) * previous_vel(2);
  // OptiState.x_dot = (alpha * (current_vel(0) + previous_vel(0)) + (1. - alpha) * OptiState.x_dot) / (alpha + 1.);
  // OptiState.y_dot = (alpha * (current_vel(1) + previous_vel(1)) + (1. - alpha) * OptiState.y_dot) / (alpha + 1.);
  // OptiState.z_dot = (alpha * (current_vel(2) + previous_vel(2)) + (1. - alpha) * OptiState.z_dot) / (alpha + 1.);
  previous_vel << current_vel;
  last_t_state_log = t1;
  // optitrack_updated = true;
}

void setupGainsHardware(const std::string filepath)
{
  YAML::Node config = YAML::LoadFile(filepath);
  p.roll_offset = config["roll_offset"].as<scalar_t>();
  p.pitch_offset = config["pitch_offset"].as<scalar_t>();

  p.orientation_kp = config["Orientation"]["Kp"].as<std::vector<scalar_t>>();
  p.orientation_kd = config["Orientation"]["Kd"].as<std::vector<scalar_t>>();
  p.leg_kp = config["Leg"]["Kp"].as<scalar_t>();
  p.leg_kd = config["Leg"]["Kd"].as<scalar_t>();
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
  p.dt_lowlevel = config["Policy"]["dt_lowlevel"].as<scalar_t>();
  p.dt_policy = config["Policy"]["dt_policy"].as<scalar_t>();
  p.horizon = config["RL"]["horizon"].as<scalar_t>();
  p.zed_x_offset = config["zed_x_offset"].as<scalar_t>();
  p.zed_y_offset = config["zed_y_offset"].as<scalar_t>();
  alpha = config["filter_alpha"].as<scalar_t>();
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

void signal_callback_handler(int signum)
{
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

  source_sock.sin_family = AF_INET;
  source_sock.sin_port = htons(port);
  source_sock.sin_addr.s_addr = inet_addr(hostname.c_str());

  destination.sin_family = AF_INET;
  destination.sin_port = htons(port);
  destination.sin_addr.s_addr = inet_addr(destname.c_str());
  destinationAddrLen = sizeof(destination);

  int enabled = 1;
  setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled));

  int val = bind(sock, (struct sockaddr *)&source_sock,
                 sizeof(source_sock));

  senderAddr.sin_family = AF_INET;
  senderAddr.sin_port = htons(port);
  senderAddr.sin_addr.s_addr = inet_addr(hostname.c_str());
  senderAddrLen = sizeof(senderAddr);
  sleep(1);
}
