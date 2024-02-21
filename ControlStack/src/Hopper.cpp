#include "../inc/Hopper.h"
#include <stdexcept>

#include <manif/manif.h>

matrix_3t Hopper::cross(vector_3t q) {
    matrix_3t c;
    c << 0, -q(2), q(1),
            q(2), 0, -q(0),
            -q(1), q(0), 0;
    return c;
}

matrix_3t Hopper::quat2Rot(quat_t q) {
    scalar_t qw,qx,qy,qz;
    matrix_3t Rq;
    qw = q.w();
    qx = q.x();
    qy = q.y();
    qz = q.z();
    Rq << pow(qw,2)+pow(qx,2)-pow(qy,2)-pow(qz,2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy),
       2*(qx*qy+qw*qz), pow(qw,2)-pow(qx,2)+pow(qy,2)-pow(qz,2),2*(qy*qz-qw*qx),
       2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), pow(qw,2)-pow(qx,2)-pow(qy,2)+pow(qz,2);
    return Rq;
}

Hopper::Hopper() {
        // Read gain yaml
    YAML::Node config = YAML::LoadFile("../config/gains.yaml");
    std::vector<scalar_t> orientation_kp = config["LowLevel"]["Orientation"]["Kp"].as<std::vector<scalar_t>>();
    std::vector<scalar_t> orientation_kd = config["LowLevel"]["Orientation"]["Kd"].as<std::vector<scalar_t>>();
    gains.leg_kp = config["LowLevel"]["Leg"]["Kp"].as<scalar_t>();
    gains.leg_kd = config["LowLevel"]["Leg"]["Kd"].as<scalar_t>();

    gains.orientation_kp << orientation_kp[0], orientation_kp[1], orientation_kp[2];
    gains.orientation_kd << orientation_kd[0], orientation_kd[1], orientation_kd[2];

    quat_actuator = quat_t(0.8806, 0.3646, -0.2795, 0.1160);
    multiplier_on_deltaf = config["MPC"]["multiplier_on_deltaf"].as<scalar_t>();

    q.resize(11);
    v.resize(10);
}

void Hopper::updateState(vector_t state) {
    int ind = 0;

    t = state[ind];
    ind++;
    pos << state[ind], state[ind + 1], state[ind + 2];
    ind += 3;
    quat = Eigen::Quaternion<scalar_t>(state[ind], state[ind + 1], state[ind + 2], state[ind + 3]); // Loaded as w,x,y,z
    ind += 4;
    vel << state[ind], state[ind + 1], state[ind + 2];
    ind += 3;
    omega << state[ind], state[ind + 1], state[ind + 2];
    ind += 3;
    if (contact == 0 && state[ind] >= .1) {
	    last_impact_time = t;
    }
    if (contact == 1 && state[ind] <= .1) {
	    last_flight_time = t;
    }
    if (state[ind] <= .1) {
            contact = 0;
    } else {
            contact = 1;
    }
    ind++;
    leg_pos = state[ind];
    ind++;
    leg_vel = state[ind];
    ind++;
    wheel_vel << state[ind], state[ind + 1], state[ind + 2];
    ind++;
    q << pos, quat.coeffs(), leg_pos, 0,0,0;
    v << vel, omega, leg_vel, wheel_vel;
};

void Hopper::computeTorque(quat_t quat_d_, vector_3t omega_d, scalar_t length_des, vector_t u_des) {

    vector_3t omega_a;

    quat_t quat_a_ = quat;

    vector_4t quat_d;
    vector_4t quat_a;
    quat_d << quat_d_.w(), quat_d_.x(), quat_d_.y(), quat_d_.z();
    quat_a << quat_a_.w(), quat_a_.x(), quat_a_.y(), quat_a_.z();

    vector_3t delta_quat;

    quat_t e = quat_d_.inverse() * quat;
    manif::SO3Tangent<scalar_t> xi;
    auto e_ = manif::SO3<scalar_t>(e);
    xi = e_.log();
    delta_quat << xi.coeffs();
     
    matrix_3t Kp, Kd;
    Kp.setZero();
    Kd.setZero();
    Kp.diagonal() << gains.orientation_kp;
    Kd.diagonal() << gains.orientation_kd;

    vector_3t tau;
    
    tau = quat_actuator.inverse()._transformVector(-Kp * delta_quat - Kd * (omega - omega_d));

    scalar_t spring_f = (1 - contact) * (-gains.leg_kp * (leg_pos - length_des) - gains.leg_kd * leg_vel);
    torque << spring_f, tau;
    torque += u_des;
};