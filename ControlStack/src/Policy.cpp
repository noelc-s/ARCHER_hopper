#include "../inc/Policy.h"
#include <manif/manif.h>

using namespace Eigen;
using namespace Hopper_t;

void Policy::loadParams(const std::string filepath, Params& params) {
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

void Policy::updateOffsets(const vector_2t offsets) {
    params.roll_d_offset = offsets[0];
    params.pitch_d_offset = offsets[1];
}

RaibertPolicy::RaibertPolicy(const std::string yamlPath) {
    loadParams(yamlPath, params);
}

quat_t RaibertPolicy::DesiredQuaternion(scalar_t x_a, scalar_t y_a, scalar_t x_d, scalar_t y_d, scalar_t xd_a, scalar_t yd_a, scalar_t yaw_des){    
    //position error
    scalar_t del_x = x_a - x_d;
    scalar_t del_y = y_a - y_d;

    // assuming pitch::x, roll::y, angle_desired = e^(k|del_pos|) - 1
    scalar_t pitch_d = std::min(params.kx_p*del_x + params.kx_d*xd_a, params.angle_max);
    pitch_d = std::max(pitch_d, -params.angle_max);
    scalar_t roll_d = std::min(params.ky_p*del_y + params.ky_d*yd_a, params.angle_max);
    roll_d = std::max(roll_d, -params.angle_max);
    static scalar_t yaw_des_rolling = 0;
    // yaw_des_rolling += yaw_damping*(yaw_des - yaw_des_rolling);
    yaw_des_rolling += params.yaw_damping*(yaw_des);
    scalar_t yaw_d = yaw_des_rolling;

    quat_t desiredLocalInput = Euler2Quaternion(roll_d - params.roll_d_offset, pitch_d - params.pitch_d_offset, yaw_d);
    
    return desiredLocalInput;

}

vector_3t RaibertPolicy::DesiredOmega(){
    vector_3t omega_des;
    omega_des << 0, 0, 0;
    return omega_des;
}

vector_4t RaibertPolicy::DesiredInputs(){
    vector_4t u_des;
    u_des << 0, 0, 0, 0;
    return u_des;
}

ZeroDynamicsPolicy::ZeroDynamicsPolicy(std::string model_name, const std::string yamlPath) {
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


void ZeroDynamicsPolicy::EvaluateNetwork(const vector_4t state, vector_2t& output) {
    
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
        memoryInfo, const_cast<float*>(input.data()), inputTensorSize,
        inputDims.data(), inputDims.size());

    Ort::Value outputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo, outpt.data(), outputTensorSize,
        outputDims.data(), outputDims.size());

    std::vector<const char*> inputNames{inputNodeName.c_str()};
    std::vector<const char*> outputNames{outputNodeName.c_str()};

    session->Run(Ort::RunOptions{}, inputNames.data(), &inputTensor, 1, outputNames.data(), &outputTensor, 1);

    output << outpt[0], outpt[1];
}

quat_t ZeroDynamicsPolicy::DesiredQuaternion(scalar_t x_a, scalar_t y_a, scalar_t x_d, scalar_t y_d, scalar_t xd_a, scalar_t yd_a, scalar_t yaw_des){

	vector_4t state;
	state << x_a - x_d, y_a - y_d, xd_a, yd_a; 
	vector_2t rp_des;
	EvaluateNetwork(state, rp_des);

    static scalar_t yaw_des_rolling = 0;
    // yaw_des_rolling += yaw_damping*(yaw_des - yaw_des_rolling);
    yaw_des_rolling += params.yaw_damping*(yaw_des);
    scalar_t yaw_d = yaw_des_rolling;

    quat_t desQuat = Euler2Quaternion(-rp_des(1) - params.roll_d_offset, rp_des(0) - params.pitch_d_offset, yaw_des_rolling);
	return desQuat;
}

vector_3t ZeroDynamicsPolicy::DesiredOmega() {
	vector_3t omega;
	omega.setZero();
	return omega;
}

vector_4t ZeroDynamicsPolicy::DesiredInputs() {
	vector_4t inputs;
	inputs.setZero();
	return inputs;
}
