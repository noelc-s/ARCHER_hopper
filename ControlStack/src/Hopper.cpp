#include "../inc/Hopper.h"
// #include "../inc/MPC.h"
#include <stdexcept>

#include <manif/manif.h>
#include "../inc/utils.h"

Hopper::Hopper(const std::string yamlFile)
{
    // Read gain yaml
    YAML::Node config = YAML::LoadFile(yamlFile);
    std::vector<scalar_t> orientation_kp = config["Orientation"]["Kp"].as<std::vector<scalar_t>>();
    std::vector<scalar_t> orientation_kd = config["Orientation"]["Kd"].as<std::vector<scalar_t>>();
    gains.leg_kp = config["Leg"]["Kp"].as<scalar_t>();
    gains.leg_kd = config["Leg"]["Kd"].as<scalar_t>();

    gains.orientation_kp << orientation_kp[0], orientation_kp[1], orientation_kp[2];
    gains.orientation_kd << orientation_kd[0], orientation_kd[1], orientation_kd[2];

    quat_actuator = quat_t(0.8806, 0.3646, -0.2795, 0.1160);

    state_.q.resize(11);
    state_.v.resize(10);

    pinocchioWrapper = createPinocchioInstance();

}

void Hopper::removeYaw(const scalar_t& optitrack_yaw)
{
    // Remove the measured yaw to put us back in the global frame
    scalar_t measured_yaw = extract_yaw(state_.quat);
    quat_t measured_yaw_quat = Euler2Quaternion(0, 0, measured_yaw);
    quat_t optitrack_yaw_quat = Euler2Quaternion(0, 0, optitrack_yaw);
    quat_t yaw_corrected = plus(optitrack_yaw_quat, minus(state_.quat, measured_yaw_quat));
    state_.quat = yaw_corrected;
}

void Hopper::updateState(vector_t state)
{
    int ind = 0;

    state_.t = state[ind];
    ind++;
    state_.pos << state[ind], state[ind + 1], state[ind + 2];
    ind += 3;
    state_.quat = Eigen::Quaternion<scalar_t>(state[ind], state[ind + 1], state[ind + 2], state[ind + 3]); // Loaded as w,x,y,z
    state_.quat.normalize();
    ind += 4;
    state_.vel << state[ind], state[ind + 1], state[ind + 2];
    ind += 3;
    state_.omega << state[ind], state[ind + 1], state[ind + 2];
    ind += 3;
    if (state_.contact == 0 && state[ind] >= .1)
    {
        state_.last_impact_time = state_.t;
    }
    if (state_.contact == 1 && state[ind] <= .1)
    {
        state_.last_flight_time = state_.t;
    }
    if (state[ind] <= .1)
    {
        state_.contact = 0;
    }
    else
    {
        state_.contact = 1;
    }
    ind++;
    state_.leg_pos = state[ind];
    ind++;
    state_.leg_vel = state[ind];
    ind++;
    state_.wheel_vel << state[ind], state[ind + 1], state[ind + 2];
    ind++;
    state_.q << state_.pos, state_.quat.coeffs(), state_.leg_pos, 0, 0, 0;
    state_.v << state_.vel, state_.omega, state_.leg_vel, state_.wheel_vel;
};

void Hopper::computeTorque(quat_t quat_d_, vector_3t omega_d, scalar_t length_des, vector_t u_des)
{

    vector_3t omega_a;

    quat_t quat_a_ = state_.quat;

    vector_4t quat_d;
    vector_4t quat_a;
    quat_d << quat_d_.w(), quat_d_.x(), quat_d_.y(), quat_d_.z();
    quat_a << quat_a_.w(), quat_a_.x(), quat_a_.y(), quat_a_.z();

    vector_3t delta_quat;
    quat_t e = quat_d_.inverse() * state_.quat;
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

    // Keep orientation control on in the ground phase
    // tau = quat_actuator.inverse()._transformVector(-Kp * delta_quat - Kd * (state_.omega - omega_d));
    // switch orientation control off in the ground phase
    tau = (1 - state_.contact) * quat_actuator.inverse()._transformVector(-Kp * delta_quat - Kd * (state_.omega - omega_d));

    scalar_t spring_f = (1 - state_.contact) * (-gains.leg_kp * (state_.leg_pos - length_des) - gains.leg_kd * state_.leg_vel);
    torque << spring_f, tau;
    torque += u_des;
};


void Hopper::css2dss(const matrix_t &Ac, const matrix_t &Bc, const matrix_t &Cc, const float dt,
                     matrix_t &Ad, matrix_t &Bd, matrix_t &Cd)
{
    // Exact discretization
    int dim = Ac.cols() + Bc.cols() + Cc.cols();
    matrix_t M = matrix_t::Zero(dim, dim);
    M << Ac, Bc, Cc, Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>::Zero(Bc.cols() + Cc.cols(), dim);
    matrix_t Me = matrix_t::Zero(dim, dim);
    M *= dt;
    Me = M.exp();
    Ad << Me.block(0, 0, Ac.rows(), Ac.cols());
    Bd << Me.block(0, Ac.cols(), Bc.rows(), Bc.cols());
    Cd << Me.block(0, Ac.cols() + Bc.cols(), Cc.rows(), Cc.cols());

    // One step euler prediction
    // Ad = (matrix_t::Identity(Ac.rows(), Ac.cols()) + Ac*dt);
    // Bd = Bc*dt;
    // Cd = Cc*dt;
};

void Hopper::DiscreteDynamics(const vector_t &x, const vector_t &u, const domain &d, const float dt,
                              matrix_t &Ac, matrix_t &Bc, matrix_t &Cc,
                              matrix_t &Ad, matrix_t &Bd, matrix_t &Cd,
                              const vector_t q0)
{
    switch (d)
    {
    case flight: case ground:
    {
        vector_t a(10);
        a << 0, 0, 0, 0, 0, 0, u;
        pinocchioWrapper->Df(x.segment(0, 11), x.segment(11, 10), a, d, Ac, Bc, Cc, q0);
        css2dss(Ac, Bc, Cc, dt, Ad, Bd, Cd);
        break;
    }
    case flight_ground: case ground_flight:
    {
        pinocchioWrapper->Ddelta_f(x.segment(0, 11), x.segment(11, 10), d, Ad, Bd, Cd, q0);
        break;
    }
    }
};
