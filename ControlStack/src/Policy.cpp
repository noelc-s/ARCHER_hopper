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

void Policy::updateOffsets(const vector_2t offsets)
{
    params.roll_d_offset = offsets[0];
    params.pitch_d_offset = offsets[1];
}

RaibertPolicy::RaibertPolicy(const std::string yamlPath)
{
    loadParams(yamlPath, params);
}

quat_t RaibertPolicy::DesiredQuaternion(scalar_t x_a, scalar_t y_a, vector_3t command, scalar_t xd_a, scalar_t yd_a, scalar_t yaw_des, bool contact)
{

    if (contact) {
        x_a = 0;
        y_a = 0;
        command(0) = xd_a;
        command(1) = yd_a;
    }

    // position error
    scalar_t del_x = x_a - command(0);
    scalar_t del_y = y_a - command(1);

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

quat_t ZeroDynamicsPolicy::DesiredQuaternion(scalar_t x_a, scalar_t y_a, vector_3t command, scalar_t xd_a, scalar_t yd_a, scalar_t yaw_des, bool contact)
{
    if (contact) {
        x_a = 0;
        y_a = 0;
        command(0) = xd_a;
        command(1) = yd_a;
    }
    vector_4t state;
    state << x_a - command(0), y_a - command(1), xd_a, yd_a;
    vector_2t rp_des;
    EvaluateNetwork(state, rp_des);

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
    if (contact) {
        u_des.segment(1,3) = -0.1 * wheel_vel;
    }
    return u_des;
}

MPCPolicy::MPCPolicy(const std::string yamlPath, std::shared_ptr<Hopper> hopper, std::shared_ptr<MPC> mpc) : hopper(std::move(hopper)), mpc_(std::move(mpc)) {
    loadParams(yamlPath, params);
    q0.resize(21);
    q0_local.resize(21);
    x_pred.resize(21,2);
    u_pred.resize(4,1);
    sol.resize(mpc_->nx * mpc_->p.N + mpc_->nu * (mpc_->p.N - 1));
    sol_g.resize((mpc_->nx + 1) * mpc_->p.N + mpc_->nu * (mpc_->p.N - 1));
    q0.setZero();
    q0_local.setZero();
    sol.setZero();
    sol_g.setZero();
    x_pred.setZero();
    u_pred.setZero();
    t_last_MPC = -1;
    dt_elapsed_MPC = 0;
}

quat_t MPCPolicy::DesiredQuaternion(scalar_t x_a, scalar_t y_a, vector_3t command, scalar_t xd_a, scalar_t yd_a, scalar_t yaw_des, bool contact)
{
    q0 << hopper->q, hopper->v;
    q0_local = MPC::global2local(q0);
    dt_elapsed_MPC = hopper->t - t_last_MPC;
    bool replan = dt_elapsed_MPC >= mpc_->p.MPC_dt_replan;
    vector_2t command_interp;
    command_interp << command.segment(0,2);
    if (replan)
    {
        mpc_->solve(*hopper, sol, command, command_interp);
        for (int i = 0; i < mpc_->p.N; i++)
        {
            sol_g.segment(i * (mpc_->nx + 1), mpc_->nx + 1) << MPC::local2global(MPC::xik_to_qk(sol.segment(i * mpc_->nx, mpc_->nx), q0_local));
        }
        sol_g.segment((mpc_->nx + 1) * mpc_->p.N, mpc_->nu * (mpc_->p.N - 1)) << sol.segment((mpc_->nx) * mpc_->p.N, mpc_->nu * (mpc_->p.N - 1));
        x_pred << MPC::local2global(MPC::xik_to_qk(sol.segment(0, 20), q0_local)), MPC::local2global(MPC::xik_to_qk(sol.segment(20, 20), q0_local));
        u_pred << sol.segment(mpc_->p.N * mpc_->nx, 4);
        t_last_MPC = hopper->t;
    }

    // Compute continuous time solution to discrete time problem.
    // vector_t x_des(21);
    // hopper.css2dss(mpc_->Ac.block(0,0,mpc_->nx,mpc_->nx),mpc_->Bc.block(0,0,mpc_->nx,mpc_->nu),mpc_->Cc.block(0,0,mpc_->nx,1),state(0)-t_last_MPC,mpc_->Ad_,mpc_->Bd_,mpc_->Cd_);
    // x_des << MPC::local2global(MPC::Exp(mpc_->Ad_*sol.segment(0,20) + mpc_->Bd_*u_pred + mpc_->Cd_));
    // quat_des = Quaternion<scalar_t>(x_des(6), x_des(3), x_des(4), x_des(5));
    // omega_des << x_des(14), x_des(15),x_des(16);

    // Simply set the next waypoint as the setpoint of the low level.
    quat_t quat_des;
    quat_des = Quaternion<scalar_t>(x_pred(6, 1), x_pred(3, 1), x_pred(4, 1), x_pred(5, 1));
    return quat_des;
}

vector_3t MPCPolicy::DesiredOmega()
{
    vector_3t omega;
    omega << x_pred(14, 1), x_pred(15, 1), x_pred(16, 1);
    return omega;
}

vector_4t MPCPolicy::DesiredInputs(const vector_3t wheel_vel, const bool contact)
{
    vector_4t inputs;
    inputs = u_pred;
    return inputs;
}

PMPPolicy::PMPPolicy(const std::string yamlPath, std::shared_ptr<Hopper> hopper, std::shared_ptr<Integrator> integrator) : 
                    hopper_(std::move(hopper)), integrator_(std::move(integrator))
{
    loadParams(yamlPath, params);
    x_sol.resize(21,num_iter);
    u_sol.resize(4,num_iter);
    x_sol.setZero();
    u_sol.setZero();
}

quat_t PMPPolicy::DesiredQuaternion(scalar_t x_a, scalar_t y_a, vector_3t command,
                scalar_t xd_a, scalar_t yd_a, scalar_t yaw_des, bool contact)
{
  vector_t x_k(21);
  vector_t u_k(4);
  vector_t p_k(20);
  scalar_t J_k = 0;
  x_k << hopper_->q, hopper_->v;
  p_k.setZero();
  vector_t x_kp1(21);
  vector_t p_kp1(20);
  scalar_t J_kp1 = 0;

  domain d;
  if (hopper_->contact) {
    d = ground;
  } else {
    d = flight;
  }

//   u_k(3) = hopper_->torque[3];
    u_k.setZero();

    std::cout << std::endl << "-----" << std::endl;
  for (int j = 0; j < num_iter; j++) {
    switch (d) {
        case flight:
            if (x_k(13) < 0 && x_k(2) <= 0.37) {
                d = flight_ground;
            }
            break;
        case flight_ground:
            d = ground;
            break;
        case ground:
            if (x_k(17) < 0 && x_k(7) <= 0) {
                d = ground_flight;
            }
            break;
        case ground_flight:
            d = flight;
            break;
    }
          std::cout << d << ":" << x_k(7) << " ";
    integrator_->xpj_integrator(hopper_, x_k, p_k, J_k, u_k, d, x_kp1, p_kp1, J_kp1);
    x_sol.block(0, j, 21, 1) << x_k;
    u_sol.block(0, j, 4, 1) << u_k;
    x_k << x_kp1;
    p_k << p_kp1;
    J_k = J_kp1;
  }
  quat_t quat_des;
  quat_des = quat_t(1, 0, 0, 0);
  return quat_des;
}

vector_3t PMPPolicy::DesiredOmega()
{
    vector_3t inputs;
    inputs.setZero();
    return inputs;
}
vector_4t PMPPolicy::DesiredInputs(const vector_3t wheel_vel, const bool contact)
{
    vector_4t inputs;
    inputs.setZero();
    return inputs;
}