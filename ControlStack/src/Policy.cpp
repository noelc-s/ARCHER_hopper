#include "../inc/Policy.h"
#include <manif/manif.h>

using namespace Eigen;
using namespace Hopper_t;

void Policy::loadParams(const std::string filepath, Params &params)
{
    YAML::Node config = YAML::LoadFile(filepath);

    scalar_t stance_time = config["MPC"]["groundDuration"].as<scalar_t>();
    params.kx_p = config["RaibertHeuristic"]["kx_p"].as<scalar_t>();
    params.ky_p = config["RaibertHeuristic"]["ky_p"].as<scalar_t>();
    params.kx_d = config["RaibertHeuristic"]["kx_d"].as<scalar_t>();
    params.ky_d = config["RaibertHeuristic"]["ky_d"].as<scalar_t>();
    params.angle_max = config["RaibertHeuristic"]["angle_max"].as<scalar_t>();
    params.pitch_d_offset = config["pitch_offset"].as<scalar_t>();
    params.roll_d_offset = config["roll_offset"].as<scalar_t>();
    params.yaw_damping = config["RaibertHeuristic"]["yaw_damping"].as<scalar_t>();
}

void RLPolicy::loadParams(const std::string filepath, RLParams &RLparams)
{
    YAML::Node config = YAML::LoadFile(filepath);
    RLparams.lin_vel_scaling = config["RL"]["obs_scales"]["lin_vel"].as<scalar_t>();
    RLparams.ang_vel_scaling = config["RL"]["obs_scales"]["ang_vel"].as<scalar_t>();
    RLparams.dof_vel_scaling = config["RL"]["obs_scales"]["dof_vel"].as<scalar_t>();
    RLparams.z_pos_scaling = config["RL"]["obs_scales"]["z_pos"].as<scalar_t>();
    RLparams.dt_replan = config["RL"]["dt_replan"].as<scalar_t>();
}

void RLTrajPolicy::loadParams(const std::string filepath, RLParams &RLparams)
{
    YAML::Node config = YAML::LoadFile(filepath);
    RLparams.lin_vel_scaling = config["RL"]["obs_scales"]["lin_vel"].as<scalar_t>();
    RLparams.ang_vel_scaling = config["RL"]["obs_scales"]["ang_vel"].as<scalar_t>();
    RLparams.dof_vel_scaling = config["RL"]["obs_scales"]["dof_vel"].as<scalar_t>();
    RLparams.z_pos_scaling = config["RL"]["obs_scales"]["z_pos"].as<scalar_t>();
    RLparams.dt_replan = config["RL"]["dt_replan"].as<scalar_t>();
}

void Policy::updateOffsets(const vector_2t offsets)
{
    params.roll_d_offset = offsets[0];
    params.pitch_d_offset = offsets[1];
}

// x_a = state(1);
// y_a = state(2);
// xd_a = state(8);
// yd_a = state(9);


RaibertPolicy::RaibertPolicy(const std::string yamlPath)
{
    loadParams(yamlPath, params);
}

quat_t RaibertPolicy::DesiredQuaternion(Hopper::State state, matrix_t command)
{
    if ((command.rows() != 3) || (command.cols() != 1)) {
        throw std::runtime_error("Input to RL Policy is not of proper shape (expected 3x1)");
    }

    bool contact = state.contact;
    scalar_t x_a = state.pos[0];
    scalar_t y_a = state.pos[1];
    scalar_t xd_a = state.vel[0];
    scalar_t yd_a = state.vel[1];

    if (contact) {
        x_a = 0;
        y_a = 0;
        command(0) = xd_a;
        command(1) = yd_a;
    }

    // position error
    scalar_t del_x = x_a - command(0);
    scalar_t del_y = y_a - command(1);
    scalar_t yaw_des = command(2);

    // assuming pitch::x, roll::y, angle_desired = e^(k|del_pos|) - 1
    scalar_t pitch_d = std::min(params.kx_p * del_x + params.kx_d * xd_a, params.angle_max);
    pitch_d = std::max(pitch_d, -params.angle_max);
    scalar_t roll_d = std::min(params.ky_p * del_y + params.ky_d * yd_a, params.angle_max);
    roll_d = std::max(roll_d, -params.angle_max);
    static scalar_t yaw_des_rolling = 0;
    // yaw_des_rolling += yaw_damping*(yaw_des - yaw_des_rolling);
    yaw_des_rolling += params.yaw_damping * (yaw_des);
    scalar_t yaw_d = yaw_des_rolling;

    quat_t desiredLocalInput = Euler2Quaternion(roll_d, pitch_d, yaw_d);

    return desiredLocalInput;
}

vector_3t RaibertPolicy::DesiredOmega()
{
    vector_3t omega_des;
    omega_des << 0, 0, 0;
    return omega_des;
}

vector_4t RaibertPolicy::DesiredInputs(const vector_3t wheel_vel, const bool contact)
{
    vector_4t u_des;
    u_des.setZero();
    if (contact) {
        u_des.segment(1,3) = -0.1 * wheel_vel;
    }
    return u_des;
}

ZeroDynamicsPolicy::ZeroDynamicsPolicy(std::string model_name, const std::string yamlPath)
{
    loadParams(yamlPath, params);

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "example-model-explorer");
    Ort::SessionOptions session_options;
    session = std::make_unique<Ort::Session>(Ort::Session(env, model_name.c_str(), session_options));

    inputNodeName = session->GetInputNameAllocated(0, allocator).get();
    outputNodeName = session->GetOutputNameAllocated(0, allocator).get();

    inputTypeInfo = std::make_unique<Ort::TypeInfo>(session->GetInputTypeInfo(0));
    auto inputTensorInfo = inputTypeInfo->GetTensorTypeAndShapeInfo();
    inputType = inputTensorInfo.GetElementType();
    inputDims = inputTensorInfo.GetShape();
    inputDims[0] = 1; // hard code batch size of 1 for evaluation

    outputTypeInfo = std::make_unique<Ort::TypeInfo>(session->GetOutputTypeInfo(0));
    auto outputTensorInfo = outputTypeInfo->GetTensorTypeAndShapeInfo();
    outputType = outputTensorInfo.GetElementType();
    outputDims = outputTensorInfo.GetShape();
    outputDims[0] = 1; // hard code batch size of 1 for evaluation

    inputTensorSize = vectorProduct(inputDims);
    outputTensorSize = vectorProduct(outputDims);
}

void ZeroDynamicsPolicy::EvaluateNetwork(const vector_4t state, vector_2t &output)
{

    std::vector<float> input(4);
    input[0] = state(0);
    input[1] = state(1);
    input[2] = state(2);
    input[3] = state(3);

    std::vector<float> outpt(2);

    auto inputTensorInfo = inputTypeInfo->GetTensorTypeAndShapeInfo();
    auto outputTensorInfo = outputTypeInfo->GetTensorTypeAndShapeInfo();

    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo, const_cast<float *>(input.data()), inputTensorSize,
        inputDims.data(), inputDims.size());

    Ort::Value outputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo, outpt.data(), outputTensorSize,
        outputDims.data(), outputDims.size());

    std::vector<const char *> inputNames{inputNodeName.c_str()};
    std::vector<const char *> outputNames{outputNodeName.c_str()};

    session->Run(Ort::RunOptions{}, inputNames.data(), &inputTensor, 1, outputNames.data(), &outputTensor, 1);

    output << outpt[0], outpt[1];
}

quat_t ZeroDynamicsPolicy::DesiredQuaternion(Hopper::State state, matrix_t command)
{
    if ((command.rows() != 3) || (command.cols() != 1)) {
        throw std::runtime_error("Input to RL Policy is not of proper shape (expected 3x1)");
    }
    bool contact = state.contact;
    scalar_t x_a = state.pos[0];
    scalar_t y_a = state.pos[1];
    scalar_t xd_a = state.vel[0];
    scalar_t yd_a = state.vel[1];

    if (contact) {
        x_a = 0;
        y_a = 0;
        command(0) = xd_a;
        command(1) = yd_a;
    }
    vector_4t input_state;
    input_state << x_a - command(0), y_a - command(1), xd_a, yd_a;
    scalar_t yaw_des = command(2);
    vector_2t rp_des;
    EvaluateNetwork(input_state, rp_des);

    static scalar_t yaw_des_rolling = 0;
    // yaw_des_rolling += yaw_damping*(yaw_des - yaw_des_rolling);
    yaw_des_rolling += params.yaw_damping * (yaw_des);
    scalar_t yaw_d = yaw_des_rolling;

    quat_t desQuat = Euler2Quaternion(-rp_des(1), rp_des(0), yaw_des_rolling);
    return desQuat;
}

vector_3t ZeroDynamicsPolicy::DesiredOmega()
{
    vector_3t omega;
    omega.setZero();
    return omega;
}

vector_4t ZeroDynamicsPolicy::DesiredInputs(const vector_3t wheel_vel, const bool contact)
{
    vector_4t u_des;
    u_des.setZero();
    vector_3t desired_wheel_vel(0, 0,0);
    if (contact) {
        u_des.segment(1,3) = -0.1 * (wheel_vel - desired_wheel_vel);
    }
    return u_des;
}

// MPCPolicy::MPCPolicy(const std::string yamlPath, std::shared_ptr<Hopper> hopper, std::shared_ptr<MPC> mpc) : hopper(std::move(hopper)), mpc_(std::move(mpc)) {
//     loadParams(yamlPath, params);
//     q0.resize(21);
//     q0_local.resize(21);
//     x_pred.resize(21,2);
//     u_pred.resize(4,1);
//     sol.resize(mpc_->nx * mpc_->p.N + mpc_->nu * (mpc_->p.N - 1));
//     sol_g.resize((mpc_->nx + 1) * mpc_->p.N + mpc_->nu * (mpc_->p.N - 1));
//     q0.setZero();
//     q0_local.setZero();
//     sol.setZero();
//     sol_g.setZero();
//     x_pred.setZero();
//     u_pred.setZero();
//     t_last_MPC = -1;
//     dt_elapsed_MPC = 0;
// }

// quat_t MPCPolicy::DesiredQuaternion(Hopper::State state, vector_3t command)
// {
//     bool contact = state.contact;
//     scalar_t x_a = state.pos[0];
//     scalar_t y_a = state.pos[1];
//     scalar_t xd_a = state.vel[0];
//     scalar_t yd_a = state.vel[1];

//     q0 << state.q, state.v;
//     q0_local = MPC::global2local(q0);
//     dt_elapsed_MPC = state.t - t_last_MPC;
//     bool replan = dt_elapsed_MPC >= mpc_->p.MPC_dt_replan;
//     vector_2t command_interp;
//     command_interp << command.segment(0,2);
//     if (replan)
//     {
//         mpc_->solve(*hopper, sol, command, command_interp);
//         for (int i = 0; i < mpc_->p.N; i++)
//         {
//             sol_g.segment(i * (mpc_->nx + 1), mpc_->nx + 1) << MPC::local2global(MPC::xik_to_qk(sol.segment(i * mpc_->nx, mpc_->nx), q0_local));
//         }
//         sol_g.segment((mpc_->nx + 1) * mpc_->p.N, mpc_->nu * (mpc_->p.N - 1)) << sol.segment((mpc_->nx) * mpc_->p.N, mpc_->nu * (mpc_->p.N - 1));
//         x_pred << MPC::local2global(MPC::xik_to_qk(sol.segment(0, 20), q0_local)), MPC::local2global(MPC::xik_to_qk(sol.segment(20, 20), q0_local));
//         u_pred << sol.segment(mpc_->p.N * mpc_->nx, 4);
//         t_last_MPC = state.t;
//     }

//     // Compute continuous time solution to discrete time problem.
//     // vector_t x_des(21);
//     // hopper.css2dss(mpc_->Ac.block(0,0,mpc_->nx,mpc_->nx),mpc_->Bc.block(0,0,mpc_->nx,mpc_->nu),mpc_->Cc.block(0,0,mpc_->nx,1),state(0)-t_last_MPC,mpc_->Ad_,mpc_->Bd_,mpc_->Cd_);
//     // x_des << MPC::local2global(MPC::Exp(mpc_->Ad_*sol.segment(0,20) + mpc_->Bd_*u_pred + mpc_->Cd_));
//     // quat_des = Quaternion<scalar_t>(x_des(6), x_des(3), x_des(4), x_des(5));
//     // omega_des << x_des(14), x_des(15),x_des(16);

//     // Simply set the next waypoint as the setpoint of the low level.
//     quat_t quat_des;
//     quat_des = Quaternion<scalar_t>(x_pred(6, 1), x_pred(3, 1), x_pred(4, 1), x_pred(5, 1));
//     return quat_des;
// }

// vector_3t MPCPolicy::DesiredOmega()
// {
//     vector_3t omega;
//     omega << x_pred(14, 1), x_pred(15, 1), x_pred(16, 1);
//     return omega;
// }

// vector_4t MPCPolicy::DesiredInputs(const vector_3t wheel_vel, const bool contact)
// {
//     vector_4t inputs;
//     inputs = u_pred;
//     return inputs;
// }


RLPolicy::RLPolicy(std::string model_name, const std::string yamlPath)
{
    loadParams(yamlPath, RLparams);
    previous_action.setZero();
    q_des << 1,0,0,0;
    t_last_RL = -1;

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "example-model-explorer");
    Ort::SessionOptions session_options;
    session = std::make_unique<Ort::Session>(Ort::Session(env, model_name.c_str(), session_options));

    inputNodeName = session->GetInputNameAllocated(0, allocator).get();
    outputNodeName = session->GetOutputNameAllocated(0, allocator).get();

    inputTypeInfo = std::make_unique<Ort::TypeInfo>(session->GetInputTypeInfo(0));
    auto inputTensorInfo = inputTypeInfo->GetTensorTypeAndShapeInfo();
    inputType = inputTensorInfo.GetElementType();
    inputDims = inputTensorInfo.GetShape();

    outputTypeInfo = std::make_unique<Ort::TypeInfo>(session->GetOutputTypeInfo(0));
    auto outputTensorInfo = outputTypeInfo->GetTensorTypeAndShapeInfo();
    outputType = outputTensorInfo.GetElementType();
    outputDims = outputTensorInfo.GetShape();

    inputTensorSize = vectorProduct(inputDims);
    outputTensorSize = vectorProduct(outputDims);
}

void RLPolicy::EvaluateNetwork(const Hopper::State state, const matrix_t command, vector_4t &output)
{
    
    std::vector<float> input(21);
    input[0] = RLparams.z_pos_scaling*state.pos[2];
    vector_4t quat_coeffs = state.quat.coeffs(); // x,y,z,w
    int quat_sign = 1; 
    if (quat_coeffs[3] < 0) {
        quat_sign = -1;
    }
    input[1] = quat_coeffs[0] * quat_sign; // x
    input[2] = quat_coeffs[1] * quat_sign; // y
    input[3] = quat_coeffs[2] * quat_sign; // z
    input[4] = quat_coeffs[3] * quat_sign; // w
    input[5] = RLparams.lin_vel_scaling*state.vel[0];
    input[6] = RLparams.lin_vel_scaling*state.vel[1];
    input[7] = RLparams.lin_vel_scaling*state.vel[2];
    input[8] = RLparams.ang_vel_scaling*state.omega[0];
    input[9] = RLparams.ang_vel_scaling*state.omega[1];
    input[10] = RLparams.ang_vel_scaling*state.omega[2];
    input[11] = RLparams.dof_vel_scaling*state.wheel_vel[0];
    input[12] = RLparams.dof_vel_scaling*state.wheel_vel[1];
    input[13] = RLparams.dof_vel_scaling*state.wheel_vel[2];
    input[14] = command(0);
    input[15] = command(1);
    input[16] = command(2);
    input[17] = previous_action[0]; // w
    input[18] = previous_action[1]; // x
    input[19] = previous_action[2]; // y
    input[20] = previous_action[3]; // z

    for (auto i : input) {
        std::cout << i << ",";
    }
    std::cout << std::endl;

    std::vector<float> outpt(4);

    auto inputTensorInfo = inputTypeInfo->GetTensorTypeAndShapeInfo();
    auto outputTensorInfo = outputTypeInfo->GetTensorTypeAndShapeInfo();

    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo, const_cast<float *>(input.data()), inputTensorSize,
        inputDims.data(), inputDims.size());

    Ort::Value outputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo, outpt.data(), outputTensorSize,
        outputDims.data(), outputDims.size());

    std::vector<const char *> inputNames{inputNodeName.c_str()};
    std::vector<const char *> outputNames{outputNodeName.c_str()};

    session->Run(Ort::RunOptions{}, inputNames.data(), &inputTensor, 1, outputNames.data(), &outputTensor, 1);

    output << outpt[0], outpt[1], outpt[2], outpt[3];

    std::cout << output.transpose() << std::endl;

}

quat_t RLPolicy::DesiredQuaternion(Hopper::State state, matrix_t command)
{
    if ((command.rows() != 3) || (command.cols() != 1)) {
        throw std::runtime_error("Input to RL Policy is not of proper shape (expected 3x1)");
    }
    dt_elapsed_RL = state.t - t_last_RL;
    bool replan = dt_elapsed_RL >= RLparams.dt_replan;
    if (replan) {
        EvaluateNetwork(state, command, q_des);
        t_last_RL = state.t;
    }
    q_des = q_des / q_des.norm(); // convention for output is w,x,y,z
    previous_action = q_des;
    if (previous_action(0) < 0) {
        previous_action = previous_action * -1;
    }
    quat_t quat_des = quat_t(q_des[0],q_des[1],q_des[2],q_des[3]); // convention for input is w,x,y,z

    return quat_des;
}

vector_3t RLPolicy::DesiredOmega()
{
    vector_3t omega;
    omega.setZero();
    return omega;
}

vector_4t RLPolicy::DesiredInputs(const vector_3t wheel_vel, const bool contact)
{
    vector_4t u_des;
    u_des.setZero();
    // vector_3t desired_wheel_vel(0, 0,0);
    // if (contact) {
    //     u_des.segment(1,3) = -0.1 * (wheel_vel - desired_wheel_vel);
    // }
    return u_des;
}



RLTrajPolicy::RLTrajPolicy(std::string model_name, const std::string yamlPath, int horizon, int state_dim): horizon(horizon), state_dim(state_dim)
{
    loadParams(yamlPath, RLparams);
    previous_action.setZero();
    q_des << 1,0,0,0;
    t_last_RL = -1;

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "example-model-explorer");
    Ort::SessionOptions session_options;
    session = std::make_unique<Ort::Session>(Ort::Session(env, model_name.c_str(), session_options));

    inputNodeName = session->GetInputNameAllocated(0, allocator).get();
    outputNodeName = session->GetOutputNameAllocated(0, allocator).get();

    inputTypeInfo = std::make_unique<Ort::TypeInfo>(session->GetInputTypeInfo(0));
    auto inputTensorInfo = inputTypeInfo->GetTensorTypeAndShapeInfo();
    inputType = inputTensorInfo.GetElementType();
    inputDims = inputTensorInfo.GetShape();

    outputTypeInfo = std::make_unique<Ort::TypeInfo>(session->GetOutputTypeInfo(0));
    auto outputTensorInfo = outputTypeInfo->GetTensorTypeAndShapeInfo();
    outputType = outputTensorInfo.GetElementType();
    outputDims = outputTensorInfo.GetShape();

    inputTensorSize = vectorProduct(inputDims);
    outputTensorSize = vectorProduct(outputDims);
}

void RLTrajPolicy::EvaluateNetwork(const Hopper::State state, const matrix_t command, vector_4t &output)
{

    std::vector<float> input(18 + horizon * state_dim);
    input[0] = RLparams.z_pos_scaling*state.pos[2];
    vector_4t quat_coeffs = state.quat.coeffs(); // x,y,z,w
    int quat_sign = 1; 
    if (quat_coeffs[3] < 0) {
        quat_sign = -1;
    }
    input[1] = quat_coeffs[0] * quat_sign; // x
    input[2] = quat_coeffs[1] * quat_sign; // y
    input[3] = quat_coeffs[2] * quat_sign; // z
    input[4] = quat_coeffs[3] * quat_sign; // w
    input[5] = RLparams.lin_vel_scaling*state.vel[0];
    input[6] = RLparams.lin_vel_scaling*state.vel[1];
    input[7] = RLparams.lin_vel_scaling*state.vel[2];
    input[8] = RLparams.ang_vel_scaling*state.omega[0];
    input[9] = RLparams.ang_vel_scaling*state.omega[1];
    input[10] = RLparams.ang_vel_scaling*state.omega[2];
    input[11] = RLparams.dof_vel_scaling*state.wheel_vel[0];
    input[12] = RLparams.dof_vel_scaling*state.wheel_vel[1];
    input[13] = RLparams.dof_vel_scaling*state.wheel_vel[2];
    
    int ind = 14;
    for (int r = 0; r < horizon; r++) {
        input[ind++] = command(r, 0) - state.pos[0];
        input[ind++] = command(r, 1) - state.pos[1];
        // if (r != 0) {
        //     std::cout << ',';
        // }
        // std::cout << std::fixed << std::setprecision(4) << command(r, 0) << ',' << command(r, 1);
        for (int c = 2; c < state_dim; c++) {
            input[ind++] = command(r, c);
            // std::cout << ',' << std::fixed << std::setprecision(4) << command(r, c);
        }
        // std::cout << "  ";
    }
    // std::cout << std::endl;
    int act_sign = 1; 
    if (previous_action[0] < 0) {  // Normalize previous action quaternion
        act_sign = -1;
    }
    input[14 + horizon * state_dim] = previous_action[0] * act_sign; // w
    input[15 + horizon * state_dim] = previous_action[1] * act_sign; // x
    input[16 + horizon * state_dim] = previous_action[2] * act_sign; // y
    input[17 + horizon * state_dim] = previous_action[3] * act_sign; // z

    // std::cout << input << std::endl << std::endl;

    // for (auto i : input) {
    //     std::cout << i << ",";
    // }
    // std::cout << std::endl << std::endl;

    std::vector<float> outpt(4);

    auto inputTensorInfo = inputTypeInfo->GetTensorTypeAndShapeInfo();
    auto outputTensorInfo = outputTypeInfo->GetTensorTypeAndShapeInfo();

    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo, const_cast<float *>(input.data()), inputTensorSize,
        inputDims.data(), inputDims.size());

    Ort::Value outputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo, outpt.data(), outputTensorSize,
        outputDims.data(), outputDims.size());

    std::vector<const char *> inputNames{inputNodeName.c_str()};
    std::vector<const char *> outputNames{outputNodeName.c_str()};

    session->Run(Ort::RunOptions{}, inputNames.data(), &inputTensor, 1, outputNames.data(), &outputTensor, 1);

    output << outpt[0], outpt[1], outpt[2], outpt[3];
}

quat_t RLTrajPolicy::DesiredQuaternion(Hopper::State state, matrix_t command)
{
    if ((command.rows() != horizon) || (command.cols() != state_dim)) {
        throw std::runtime_error("Input to RL Traj Policy is not of proper shape (expected 10x2)");
    }
    dt_elapsed_RL = state.t - t_last_RL;
    bool replan = dt_elapsed_RL >= RLparams.dt_replan;
    if (replan) {
        EvaluateNetwork(state, command, q_des);
        t_last_RL = state.t;
    }
    q_des = q_des / q_des.norm(); // convention for output is w,x,y,z
    previous_action = q_des;
    quat_t quat_des = quat_t(q_des[0],q_des[1],q_des[2],q_des[3]); // convention for input is w,x,y,z

    return quat_des;
}

vector_3t RLTrajPolicy::DesiredOmega()
{
    vector_3t omega;
    omega.setZero();
    return omega;
}

vector_4t RLTrajPolicy::DesiredInputs(const vector_3t wheel_vel, const bool contact)
{
    vector_4t u_des;
    u_des.setZero();
    // vector_3t desired_wheel_vel(0, 0,0);
    // if (contact) {
    //     u_des.segment(1,3) = -0.1 * (wheel_vel - desired_wheel_vel);
    // }
    return u_des;
}