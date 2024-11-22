#include "../inc/Hopper.h"
// #include "../inc/MPC.h"
#include <stdexcept>

#include <manif/manif.h>
#include "../inc/utils.h"

// #include "pinocchio/algorithm/cholesky.hpp"
// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/algorithm/aba.hpp"
// #include "pinocchio/algorithm/aba-derivatives.hpp"
// #include "pinocchio/algorithm/constrained-dynamics-derivatives.hpp"
// #include "pinocchio/algorithm/impulse-dynamics.hpp"
// #include "pinocchio/algorithm/impulse-dynamics-derivatives.hpp"
// #include "pinocchio/algorithm/constrained-dynamics.hpp"

// using namespace pinocchio;

Hopper::Hopper(const std::string yamlFile)
{
    // Construct Pinocchio model
    // const std::string urdf_path_c = "../rsc/hopper.urdf";
    // model = pinocchio::Model();
    // pinocchio::urdf::buildModel(urdf_path_c, pinocchio::JointModelFreeFlyer(), model);
    // data = Data(model);

    // contact_model_ground.emplace_back(RigidConstraintModelTpl<scalar_t, 0>(CONTACT_3D, model, 2, SE3::Identity(), LOCAL_WORLD_ALIGNED));
    // contact_data_ground.emplace_back(RigidConstraintDataTpl<scalar_t, 0>(contact_model_ground.at(0)));
    // initConstraintDynamics(model, data, contact_model_ground);

    // // Contact for when foot hits hard stops going to flight phase
    // contact_model_flight.emplace_back(RigidConstraintModelTpl<scalar_t, 0>(CONTACT_3D, model, 2,LOCAL));
    // contact_data_flight.emplace_back(RigidConstraintDataTpl<scalar_t, 0>(contact_model_flight.at(0)));
    // initConstraintDynamics(model, data, contact_model_flight);

    // Read gain yaml
    YAML::Node config = YAML::LoadFile(yamlFile);
    std::vector<scalar_t> orientation_kp = config["Orientation"]["Kp"].as<std::vector<scalar_t>>();
    std::vector<scalar_t> orientation_kd = config["Orientation"]["Kd"].as<std::vector<scalar_t>>();
    gains.leg_kp = config["Leg"]["Kp"].as<scalar_t>();
    gains.leg_kd = config["Leg"]["Kd"].as<scalar_t>();
    gains.orientation_kp << orientation_kp[0], orientation_kp[1], orientation_kp[2];
    gains.orientation_kd << orientation_kd[0], orientation_kd[1], orientation_kd[2];

    springStiffness = config["SpringStiffness"].as<scalar_t>();
    quat_actuator = quat_t(0.8806, 0.3646, -0.2795, 0.1160);
    multiplier_on_deltaf = 0; // config["MPC"]["multiplier_on_deltaf"].as<scalar_t>();

    state_.q.resize(11);
    state_.v.resize(10);
    
    zeroState();
}

void Hopper::zeroState() {
    state_.q.setZero();
    state_.v.setZero();
    state_.quat.setIdentity();
    state_.pos.setZero();
    state_.vel.setZero();
    state_.omega.setZero();
    state_.leg_pos = 0;
    state_.leg_vel = 0;
    state_.t = 0;
    state_.contact = 0;
    state_.last_flight_time = 0;
    state_.wheel_vel.setZero();
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

    // std::cout << state[ind] << ", " << state_.contact << std::endl;
    // For trampoline, register contact when z velocity is zero
    // // TODO: make this a bool
    // if (state_.contact > 0.5) {
    //   if (state_.vel[2] >= 0) {
	//     state_.contact = 1;
    //   } else {
	//     state_.contact = 0;
    //   }
    // }
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
    // if (quat_d_.coeffs().transpose().array().isNaN().any()) {
    //     quat_d_.setIdentity();
    // }
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

// vector_t Hopper::f(const vector_t& q, const vector_t& v, const vector_t& a, const domain& d) {
//     vector_t x_dot(2*model.nv);
//     switch(d)
//     {
//         case flight: {
// 		//Not constrained dynamics for flight, because that would fix the foot position
// 		aba(model, data, q, v, a);
//     		x_dot << v.segment(0,6),0,v.segment(7,3), data.ddq;
// 		break;
// 	}
//         case ground: {
//                 initConstraintDynamics(model, data, contact_model_ground);
// 		constraintDynamics(model, data, q, v, a, contact_model_ground, contact_data_ground);
// 		vector_t springForce(10);
// 		springForce << 0,0,springStiffness*q(7),0,0,0,-springStiffness*q(7),0,0,0;
//     		x_dot << v, data.ddq + springForce;
// 		break;
// 	}
// 	otherwise: {
// 	     throw std::invalid_argument("Invalid domain in f");
// 	     break;
// 	}
//     }
//     quat_t quat(q(6), q(3), q(4), q(5));
//     x_dot.segment(0,3) += cross(q.segment(0,3))*quat.inverse()._transformVector(v.segment(3,3));
//     return x_dot;
// };

// void Hopper::Df(const vector_t q, const vector_t v, const vector_t a, const domain d,
//                 matrix_t &A, matrix_t &B, matrix_t &C, const vector_t q0) {
// 	matrix_t springJacobian(10,10);
// 	// Call dynamics first to populate q, v, a, for ComputeConstraintDynamicsDerivatives
// 	vector_t f = Hopper::f(q, v, a, d);
//     switch(d)
//     {
//         case flight: {
// 		//Not constrained dynamics for flight, because that would fix the foot position
// 		computeABADerivatives(model, data, q,v,a);
// 		springJacobian.setZero();
// 		break;
// 	}
//         case ground: {
//                 initConstraintDynamics(model, data, contact_model_ground);
// 		computeConstraintDynamicsDerivatives(model, data, contact_model_ground, contact_data_ground);
// 		springJacobian << matrix_t::Zero(2,10),
// 		       0,0,0,0,0,0,springStiffness,0,0,0,
// 		       matrix_t::Zero(3,10),
// 		       0,0,0,0,0,0,-springStiffness,0,0,0,
// 		       matrix_t::Zero(3,10);
// 		break;
//         }
// 	otherwise: {
// 	     throw std::invalid_argument("Invalid domain in Df");
// 	     break;
// 	}
//     }
//     A << matrix_t::Zero(10,10),matrix_t::Identity(10,10), data.ddq_dq+springJacobian, data.ddq_dv;

//     matrix_t B_mat(10,4);
//     B_mat << 0,0,0,0,
//         0,0,0,0,
//         0,0,0,0,
//         0,0,0,0,
//         0,0,0,0,
//         0,0,0,0,
//         1,0,0,0,
//         0,1,0,0,
//         0,0,1,0,
//         0,0,0,1;
//     B << matrix_t::Zero(10,4), data.Minv*B_mat;

//     vector_t s(20);
//     vector_t x(21);
//     x << q,v;
//     s = qk_to_xik(x,q0);
	
//     C << f - A * s - B * a.tail(4);
// };

// void Hopper::css2dss(const matrix_t &Ac, const matrix_t &Bc, const matrix_t &Cc, const float dt,
//                                            matrix_t &Ad, matrix_t &Bd, matrix_t &Cd) {
//     // Exact discretization
//     int dim = Ac.cols() + Bc.cols() + Cc.cols();
//     matrix_t M = matrix_t::Zero(dim, dim);
//     M << Ac, Bc, Cc, Eigen::Matrix<scalar_t, Eigen::Dynamic,Eigen::Dynamic>::Zero( Bc.cols() + Cc.cols(), dim);
//     matrix_t Me = matrix_t::Zero(dim, dim);
//     M *= dt;
//     Me = M.exp();
//     Ad << Me.block(0, 0, Ac.rows(), Ac.cols());
//     Bd << Me.block(0, Ac.cols(), Bc.rows(), Bc.cols());
//     Cd << Me.block(0, Ac.cols() + Bc.cols(), Cc.rows(), Cc.cols());

//     // One step euler prediction
//     //Ad = (matrix_t::Identity(Ac.rows(), Ac.cols()) + Ac*dt);
//     //Bd = Bc*dt;
//     //Cd = Cc*dt;
// };

// vector_t Hopper::delta_f(const vector_t q, const vector_t v, const domain d){
//     const double r_coeff = 0; // restitution coeff -- assumes perfectly plastic
//     const ProximalSettingsTpl<double> settings; // default has mu = 0
//     switch(d)
//     {
//         case flight_ground: {
//           initConstraintDynamics(model, data, contact_model_ground);
//           impulseDynamics(model, data, q, v, contact_model_ground, contact_data_ground, r_coeff, settings);
// 	  break;
//         }
// 	case ground_flight: {
//           initConstraintDynamics(model, data, contact_model_flight);
//           impulseDynamics(model, data, q, v, contact_model_flight, contact_data_flight, r_coeff, settings);
// 	  break;
// 	}
//     }
//     vector_t x_plus(21); 
//     x_plus << q, (1-multiplier_on_deltaf)*v + multiplier_on_deltaf*data.dq_after;
//     return x_plus;
// };

// void Hopper::Ddelta_f(const vector_t q, const vector_t v, const domain d,
//                       matrix_t &A, matrix_t &B, matrix_t &C, const vector_t q0){
//     vector_t df(21); 
//     const double r_coeff = 0; // restitution coeff -- assumes perfectly plastic
//     const ProximalSettingsTpl<double> settings; // default has mu = 0
//     switch(d)
//     {
//         case flight_ground: {
//           df = Hopper::delta_f(q, v, d);
//           initConstraintDynamics(model, data, contact_model_ground);
//           computeImpulseDynamicsDerivatives(model, data, contact_model_ground, contact_data_ground, r_coeff, settings);
// 	  break;
// 	}
// 	case ground_flight: {
//           df = Hopper::delta_f(q, v, d);
//           initConstraintDynamics(model, data, contact_model_flight);
//           computeImpulseDynamicsDerivatives(model, data, contact_model_flight, contact_data_flight, r_coeff, settings);
// 	  break;
// 	}
//     }
//     A << matrix_t::Identity(10,10),matrix_t::Zero(10,10), data.ddq_dq, data.ddq_dv;

//     B.setZero();

//     vector_t s(20);
//     vector_t x(21);
//     x << q,v;
//     s = qk_to_xik(x,q0);

//     vector_t s_df(20);
//     s_df << s.segment(0,10),df.segment(11,10);

//     C << s_df - A * s;
// };

// void Hopper::DiscreteDynamics(const vector_t &x, const vector_t &u, const domain &d, const float dt,
//                               matrix_t &Ac, matrix_t &Bc, matrix_t &Cc,
//                               matrix_t &Ad, matrix_t &Bd, matrix_t &Cd,
// 			      const vector_t q0) {
//     switch(d)
//     {
//         case flight: {
//             vector_t a(10);
//             a << 0, 0, 0, 0, 0, 0, u;
//             Df(x.segment(0, 11), x.segment(11, 10), a, d, Ac, Bc, Cc,q0);
//             css2dss(Ac, Bc, Cc, dt, Ad, Bd, Cd);
//             break;
//         }
//         case ground: {
//             vector_t a(10);
//             a << 0, 0, 0, 0, 0, 0, u;
//             Df(x.segment(0, 11), x.segment(11, 10), a, d, Ac, Bc, Cc,q0);
//             css2dss(Ac, Bc, Cc, dt, Ad, Bd, Cd);
//             break;
//         }
//         case flight_ground: {
//             Ddelta_f(x.segment(0, 11), x.segment(11, 10), d, Ad, Bd, Cd,q0);
//             break;
//         }
//         case ground_flight: {
//             Ddelta_f(x.segment(0, 11), x.segment(11, 10), d, Ad, Bd, Cd,q0);
//             break;
//         }
//     }
// };

// NNHopper::NNHopper(std::string model_name, const std::string yamlPath) : Hopper(yamlPath)
// {
//     Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "example-model-explorer");
//     Ort::SessionOptions session_options;
//     session = std::make_unique<Ort::Session>(Ort::Session(env, model_name.c_str(), session_options));

//     inputNodeName = session->GetInputNameAllocated(0, allocator).get();
//     outputNodeName = session->GetOutputNameAllocated(0, allocator).get();

//     inputTypeInfo = std::make_unique<Ort::TypeInfo>(session->GetInputTypeInfo(0));
//     auto inputTensorInfo = inputTypeInfo->GetTensorTypeAndShapeInfo();
//     inputType = inputTensorInfo.GetElementType();
//     inputDims = inputTensorInfo.GetShape();
//     // inputDims[0] = 1; // hard code batch size of 1 for evaluation

//     outputTypeInfo = std::make_unique<Ort::TypeInfo>(session->GetOutputTypeInfo(0));
//     auto outputTensorInfo = outputTypeInfo->GetTensorTypeAndShapeInfo();
//     outputType = outputTensorInfo.GetElementType();
//     outputDims = outputTensorInfo.GetShape();
//     // outputDims[0] = 1; // hard code batch size of 1 for evaluation

//     inputTensorSize = vectorProduct(inputDims);
//     outputTensorSize = vectorProduct(outputDims);
// }

// void NNHopper::EvaluateNetwork(const quat_t quat_err, const vector_3t omega, const vector_3t flywheel_speed, vector_3t &tau)
// {
//     std::vector<float> input(10);
//     input[0] = quat_err.w();
//     input[1] = quat_err.x();
//     input[2] = quat_err.y();
//     input[3] = quat_err.z();
//     input[4] = omega(0);
//     input[5] = omega(1);
//     input[6] = omega(2);
//     input[7] = flywheel_speed(0);
//     input[8] = flywheel_speed(1);
//     input[9] = flywheel_speed(2);

//     std::vector<double> outpt(3);

//     auto inputTensorInfo = inputTypeInfo->GetTensorTypeAndShapeInfo();
//     auto outputTensorInfo = outputTypeInfo->GetTensorTypeAndShapeInfo();
//     Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
//         OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
//     Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
//         memoryInfo, const_cast<float *>(input.data()), inputTensorSize,
//         inputDims.data(), inputDims.size());
//     Ort::Value outputTensor = Ort::Value::CreateTensor<double>(
//         memoryInfo, outpt.data(), outputTensorSize,
//         outputDims.data(), outputDims.size());

//     std::vector<const char *> inputNames{inputNodeName.c_str()};
//     std::vector<const char *> outputNames{outputNodeName.c_str()};
//     session->Run(Ort::RunOptions{}, inputNames.data(), &inputTensor, 1, outputNames.data(), &outputTensor, 1);

//     tau << outpt[0], outpt[1], outpt[2];
// }

// void NNHopper::computeTorque(quat_t quat_d_, vector_3t omega_d, scalar_t length_des, vector_t u_des)
// {

//     vector_3t tau, body_axis_aligned_torque, NN_output;

//     quat_t q_diff = quat_d_.inverse() * state_.quat;
//     vector_3t omega_diff = state_.omega - omega_d;

//     EvaluateNetwork(q_diff, state_.omega, state_.wheel_vel, NN_output);

//     body_axis_aligned_torque = NN_output;

//     tau = quat_actuator.inverse()._transformVector(body_axis_aligned_torque);

//     scalar_t spring_f = (1 - state_.contact) * (-gains.leg_kp * (state_.leg_pos - length_des) - gains.leg_kd * state_.leg_vel);
//     torque << spring_f, tau;
//     torque += u_des;
// };
