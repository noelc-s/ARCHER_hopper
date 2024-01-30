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

quat_t Policy::Euler2Quaternion(scalar_t roll, scalar_t pitch, scalar_t yaw){
    quat_t q = AngleAxisd(roll, Vector3d::UnitX())
                    * AngleAxisd(pitch, Vector3d::UnitY())
                    * AngleAxisd(yaw, Vector3d::UnitZ());
    
    return q;
                       
}

template <typename T> int Policy::sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

quat_t Policy::DesiredQuaternion(scalar_t x_a, scalar_t y_a, scalar_t x_d, scalar_t y_d, scalar_t xd_a, scalar_t yd_a, vector_3t currentEulerAngles){
    
    //scaling coefficients for the exponential map
    YAML::Node config = YAML::LoadFile("../config/gains.yaml");
    scalar_t stance_time = config["MPC"]["groundDuration"].as<scalar_t>();

    scalar_t kx_p = config["RaibertHeuristic"]["kx_p"].as<scalar_t>();
    scalar_t ky_p = config["RaibertHeuristic"]["ky_p"].as<scalar_t>();
    scalar_t kx_d = config["RaibertHeuristic"]["kx_d"].as<scalar_t>();
    scalar_t ky_d = config["RaibertHeuristic"]["ky_d"].as<scalar_t>();
    scalar_t angle_max = config["RaibertHeuristic"]["angle_max"].as<scalar_t>();

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
    scalar_t pitch_d = std::min(kx_p*del_x + kx_d*xd_a, angle_max);
    pitch_d = std::max(pitch_d, -angle_max);
    scalar_t roll_d = std::min(ky_p*del_y + ky_d*yd_a, angle_max);
    roll_d = std::max(roll_d, -angle_max);
    scalar_t yaw_d = 0;

// std::cout << "Desired Pitch: " << pitch_d << std::endl;
// std::cout << "Desired Roll: " << roll_d << std::endl;

    vector_3t desiredEulerAngles;
    desiredEulerAngles << roll_d, pitch_d, yaw_d;

    quat_t desiredLocalInput = YawTransformation(currentEulerAngles, desiredEulerAngles);
    
    return desiredLocalInput;

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


quat_t Policy::MultiplyQuaternions(quat_t input, quat_t multiplier){
    quat_t q;

    q = (multiplier)*(input)*(multiplier.inverse());

    return q;
}


quat_t Policy::YawTransformation(vector_3t currentEulerAngles, vector_3t desiredEulerAngles){
    // EulerAngles == roll, pitch, yaw 

    // generating the quaternion map from current yaw only for mapping back and forth
    quat_t currentYawQuaternionInverse;
    currentYawQuaternionInverse = Euler2Quaternion(0, 0, currentEulerAngles(2)).inverse();
    // Quaternionf currentYawQuaternionInverse = currentYawQuaternion.inverse();

    // converting the desired euler angles for quaternion multiplication [0, roll, pitch, yaw]
    quat_t desiredEulerAnglesQuatRep;
    desiredEulerAnglesQuatRep.w() = 0;
    desiredEulerAnglesQuatRep.vec() = desiredEulerAngles;

    // changing to local basis
    quat_t inputLocalBasis = MultiplyQuaternions(desiredEulerAnglesQuatRep, currentYawQuaternionInverse);
    
    return inputLocalBasis;

}