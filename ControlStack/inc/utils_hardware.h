#pragma once

#include "../inc/Types.h"
#include <condition_variable>
#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "../inc/Hopper.h"

#include "../inc/estimatedState.h"

#include <librealsense2/rs.hpp>

using namespace Hopper_t;

char send_buff[40];
float desstate[10];
char buff[52];
float states[13];
vector_t ESPstate(13);
volatile bool ESP_initialized = false;
const vector_3t r_cam_to_body = {-0.067, -0.05, 0.121};
// const vector_3t r_cam_to_body = {-0.134, -0.086, 0.066};

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
  scalar_t filter_v_alpha;
  scalar_t filter_w_alpha;
} p;

std::mutex state_mtx;
std::mutex des_state_mtx;
int sock;
sockaddr_in source_sock, destination;
sockaddr_in senderAddr;
socklen_t destinationAddrLen;
socklen_t senderAddrLen;

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

void realSenseLoop(scalar_t& yaw, EstimatedState& estimated_state, bool& realsense_connected, bool& reset_pos) {

    Eigen::Quaternion<double> q;  
    Eigen::Matrix<double, 3, 1> v;
    Eigen::Matrix<double, 3, 3> R_RS_to_RS_aligned;
    Eigen::Matrix<double, 3, 3> R_RS_aligned_to_H;
    Eigen::Matrix<double, 3, 3> R_H_to_RS;
    Eigen::Matrix<double, 3, 3> R_z_up;                
    Eigen::Matrix<double, 3, 3> R_error;

    vector_3t realsense_pos{0,0,0};
    vector_3t pos_origin{0,0,0};
    vector_3t realsense_vel{0,0,0};
    vector_3t realsense_ang_vel{0,0,0};
    quat_t realsense_q;
    
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

    // Magic numbers from solidworks macro
    R_RS_to_RS_aligned << 0.271397251061513,    0.863510292339979,  0.425080588993638,
                          0.962467418729722,   -0.243493249790936, -0.119864528489434,
                          1.01307850997046E-15, 0.441657120772634, -0.897183920760302;
    R_RS_aligned_to_H << 0, 1, 0,
                         0, 0, 1,
                         1, 0, 0;
    R_error <<0.9995,    0.0000,    0.0305,
              0.0006,    0.9998,   -0.0194,
             -0.0305,    0.0194,    0.9993;
    R_H_to_RS = R_error.transpose() * R_RS_to_RS_aligned * R_RS_aligned_to_H;

    R_z_up << 1,0,0,
              0,0,-1,
              0,1,0;
    
    auto callback = [&](const rs2::frame& frame)
    {
        if (rs2::pose_frame fp = frame.as<rs2::pose_frame>()) {
            // Collect pose data
            rs2_pose pose_data = fp.get_pose_data();
            // Compensate for system lag
            auto now = std::chrono::system_clock::now().time_since_epoch();
            double now_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
            double pose_time_ms = fp.get_timestamp();
            float dt_s = static_cast<float>(std::max(0., (now_ms - pose_time_ms)/1000.));
            // std::cout << dt_s << std::endl;
            rs2_pose predicted_pose = predict_pose(pose_data, dt_s);  // Question: how much lag are we typically compensating for here -> 0.005s ~ one frame
            
            // Realsense position, velocity, angular velocity (in camera frame)
            realsense_pos << predicted_pose.translation.x,
                             predicted_pose.translation.y,
                             predicted_pose.translation.z;
            realsense_vel << predicted_pose.velocity.x,
                             predicted_pose.velocity.y,
                             predicted_pose.velocity.z;
            realsense_ang_vel << predicted_pose.angular_velocity.x,
                                 predicted_pose.angular_velocity.y,
                                 predicted_pose.angular_velocity.z;
            
            // realsense orientation
            q = Eigen::Quaternion<double>(pose_data.rotation.w, pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.z);

            // Transform velocities into local frame
            realsense_vel = R_H_to_RS.transpose()*(q.inverse() * realsense_vel);
            realsense_ang_vel = R_H_to_RS.transpose()*(q.inverse() * realsense_ang_vel); // transform to local vel

            // Body orientation (correct for where identity quat is, and cam to body transform R)
            quat_t body_q = quat_t(R_z_up) * q * quat_t(R_H_to_RS);

            // Remove initial yaw (continuing to correct global identity)
            static scalar_t initial_yaw = extract_yaw(body_q);
            quat_t inv_yaw_quat = Euler2Quaternion(0,0,-initial_yaw);
            body_q = inv_yaw_quat * body_q;

            // Transform position into the global frame
            vector_t camera_pos = inv_yaw_quat * quat_t(R_z_up) * realsense_pos;    // Camera pos with z up (and initial yaw removed)
            static vector_t p0  = camera_pos + body_q * r_cam_to_body;              // Hopper initial position global frame
            vector_t global_pos = camera_pos + body_q * r_cam_to_body - p0;         // Hopper current position global frame
            // std::cout << camera_pos.transpose() << "," <<  global_pos.transpose() << std::endl;
            
            // Print camera and gloal positions
            // std::cout << "Camera pos: " << camera_pos.transpose() << "    global pos: " << global_pos.transpose() << std::endl;
            // Reset origin
            if (reset_pos) {
                pos_origin = global_pos;
                std::cout << "Reset command received" << std::endl;
                reset_pos = false;
            }

            estimated_state.cam_q_w = q.w();                            // Quat from camera
            estimated_state.cam_q_x = q.x();
            estimated_state.cam_q_y = q.y();
            estimated_state.cam_q_z = q.z();
            estimated_state.q_w = body_q.w();                           // body quaternion, with initial yaw removed
            estimated_state.q_x = body_q.x();
            estimated_state.q_y = body_q.y();
            estimated_state.q_z = body_q.z();
            estimated_state.x = global_pos(0) - pos_origin(0);          // global position (initial yaw removed)
            estimated_state.y = global_pos(1) - pos_origin(1);
            estimated_state.z = global_pos(2) - pos_origin(2);
            estimated_state.cam_x = camera_pos(0) - pos_origin(0);      // camera position (global frame, initial yaw removed)
            estimated_state.cam_y = camera_pos(1) - pos_origin(1);
            estimated_state.cam_z = camera_pos(2) - pos_origin(2);
            estimated_state.x_dot = realsense_vel(0);                   // linear velocity, body frame
            estimated_state.y_dot = realsense_vel(1);
            estimated_state.z_dot = realsense_vel(2);
            estimated_state.omega_x = realsense_ang_vel(0);             // angular velocity, body frame
            estimated_state.omega_y = realsense_ang_vel(1);
            estimated_state.omega_z = realsense_ang_vel(2);
        }
    };

    rs2::pipeline_profile profiles = pipe.start(cfg, callback);
    std::cout << "started RealSense\n";
    realsense_connected = true;
    while(true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // why are we running this at 10Hz? and is RealSense == t265?
    }
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
    p.filter_v_alpha = config["filter_v_alpha"].as<scalar_t>();
    p.filter_w_alpha = config["filter_w_alpha"].as<scalar_t>();
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

