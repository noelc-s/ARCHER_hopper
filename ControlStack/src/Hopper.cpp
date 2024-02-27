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

NNHopper::NNHopper(std::string model_name, const std::string yamlPath) {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "example-model-explorer");
    Ort::SessionOptions session_options;
    session = std::make_unique<Ort::Session>(Ort::Session(env, model_name.c_str(), session_options));

    inputNodeName = session->GetInputNameAllocated(0, allocator).get();
    outputNodeName = session->GetOutputNameAllocated(0, allocator).get();

    inputTypeInfo = std::make_unique<Ort::TypeInfo>(session->GetInputTypeInfo(0));
    auto inputTensorInfo = inputTypeInfo->GetTensorTypeAndShapeInfo();
    inputType = inputTensorInfo.GetElementType();
    inputDims = inputTensorInfo.GetShape();
    // inputDims[0] = 1; // hard code batch size of 1 for evaluation

    outputTypeInfo = std::make_unique<Ort::TypeInfo>(session->GetOutputTypeInfo(0));
    auto outputTensorInfo = outputTypeInfo->GetTensorTypeAndShapeInfo();
    outputType = outputTensorInfo.GetElementType();
    outputDims = outputTensorInfo.GetShape();
    // outputDims[0] = 1; // hard code batch size of 1 for evaluation

    inputTensorSize = vectorProduct(inputDims);
    outputTensorSize = vectorProduct(outputDims);
}

void NNHopper::EvaluateNetwork(const vector_3t rpy_err, const vector_3t omega, const vector_3t flywheel_speed, vector_3t& tau) {
    
    std::vector<double> input(9);
    input[0] = rpy_err(0);
    input[1] = rpy_err(1);
    input[2] = rpy_err(2);
    input[3] = omega(0);
    input[4] = omega(1);
    input[5] = omega(2);
    input[6] = flywheel_speed(0);
    input[7] = flywheel_speed(1);
    input[8] = flywheel_speed(2);

    std::vector<double> outpt(3);

    auto inputTensorInfo = inputTypeInfo->GetTensorTypeAndShapeInfo();
    auto outputTensorInfo = outputTypeInfo->GetTensorTypeAndShapeInfo();

    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    Ort::Value inputTensor = Ort::Value::CreateTensor<double>(
        memoryInfo, const_cast<double*>(input.data()), inputTensorSize,
        inputDims.data(), inputDims.size());

    Ort::Value outputTensor = Ort::Value::CreateTensor<double>(
        memoryInfo, outpt.data(), outputTensorSize,
        outputDims.data(), outputDims.size());

    std::vector<const char*> inputNames{inputNodeName.c_str()};
    std::vector<const char*> outputNames{outputNodeName.c_str()};

    session->Run(Ort::RunOptions{}, inputNames.data(), &inputTensor, 1, outputNames.data(), &outputTensor, 1);

    tau << outpt[0], outpt[1], outpt[2];
}

void NNHopper::computeTorque(quat_t quat_d_, vector_3t omega_d, scalar_t length_des, vector_t u_des) {

    vector_3t tau, body_axis_aligned_torque, NN_output;

    quat_t q_diff = quat_d_.inverse() * quat;
    vector_3t omega_diff = omega - omega_d;

    EvaluateNetwork(Policy::Quaternion2Euler(q_diff), omega, wheel_vel, NN_output); 

    body_axis_aligned_torque = NN_output;
    
    tau = quat_actuator.inverse()._transformVector(body_axis_aligned_torque);

    scalar_t spring_f = (1 - contact) * (-gains.leg_kp * (leg_pos - length_des) - gains.leg_kd * leg_vel);
    torque << spring_f, tau;
    torque += u_des;
};