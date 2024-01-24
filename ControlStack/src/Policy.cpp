#include "../inc/Policy.h"

// x_d = 0
// xdot_d = 0
// y_d = 0
// ydot_d = 0

// rdot_d = 0
// pdot_d = 0
// ydot_d = 0

// control algorithm
// 1. Desired Forward velocity
// xf_0 = xdot*T/2         xdot = known, T = unknown; can be calculated from the stance phase from the previous iteration. What about the first iteration?
// xf_delta = kx*(xdot - xdot_des) = kx(xdot)
// xf = xf_0 + xf_delta
// Inverse Map: Foot Position to (r, p, y, l) <- this gives the desired Roll, Pitch, Yaw and length (r_d, p_d, y_d, l_d)
// RPY2Quat(r_d, p_d, y_d) -> (q0_d, q1_d, q2_d, q3_d)
// Quaternion2FlywheelAngles(q0_d, q1_d, q2_d, q3_d) -> (theta_1, theta_2, theta_3)


// 2. Desired Attitude
// 3. Desired Height
using namespace Eigen;
using namespace Hopper_t;


quat_t Policy::eulerToQuaternion(scalar_t roll, scalar_t pitch, scalar_t yaw) {
    
    quat_t quaternion = AngleAxisd(roll, Vector3d::UnitX())
                      * AngleAxisd(pitch, Vector3d::UnitY())
                      * AngleAxisd(yaw, Vector3d::UnitZ());

// std::cout<<"Quaternion"<<quaternion.w()<<std::endl<<quaternion.x()<<std::endl<<quaternion.y()<<std::endl<<quaternion.z()<<std::endl;
    std::cout<<quaternion.coeffs()<<std::endl;
    return quaternion;
}

template <typename T> int Policy::sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

quat_t Policy::DesiredQuaternion(scalar_t x_a, scalar_t y_a, scalar_t x_d, scalar_t y_d){
    
    //scaling coefficients for the exponential map
    YAML::Node config = YAML::LoadFile("../config/gains.yaml");
    scalar_t stance_time = config["MPC"]["groundDuration"].as<scalar_t>();

    scalar_t kx = config["RaibertHeuristic"]["kx"].as<scalar_t>();
    scalar_t ky = config["RaibertHeuristic"]["ky"].as<scalar_t>();

// quat_t quat_des = Eigen::Quaternion<scalar_t>(1,0,0,0);

// scalar_t xdot = states(8); 
// scalar_t ydot = states(9);

// xf_0 = (xdot*T)/2;
// yf_0 = (ydot*T)/2;

// xf = xf_0 + kx(xdot - xdot_d);
// yf = yf_0 + kx(ydot - ydot_d);// std::cout << "Desired Pitch: " << pitch_d << std::endl;
// std::cout << "Desired Roll: " << roll_d << std::endl;
    
    //position error
    scalar_t del_x = x_a - x_d;
    scalar_t del_y = y_a - y_d;
// std::cout<<sgn(del_x);

    // assuming pitch::x, roll::y, angle_desired = e^(k|del_pos|) - 1
    scalar_t pitch_d = (exp(kx*sgn(del_x)*del_x) - 1);
    scalar_t roll_d = (exp(ky*sgn(del_y)*del_y) - 1);
    scalar_t yaw_d = 0;

// std::cout << "Desired Pitch: " << pitch_d << std::endl;
// std::cout << "Desired Roll: " << roll_d << std::endl;

    quat_t quat_d = eulerToQuaternion(roll_d, pitch_d, yaw_d);

    return quat_d;

}


vector_3t Policy::DesiredOmega(){
   
    vector_3t omega_des;
    omega_des << 0, 0, 0;
    
    return omega_des;

}

vector_4t Policy::DesiredInputs(){

    vector_4t u_des;
    u_des << 0, 0, 0, 0;
    
    return u_des;
}