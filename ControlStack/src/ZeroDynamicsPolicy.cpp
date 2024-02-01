#include "../inc/ZeroDynamicsPolicy.h"

// Based on:
// https://github.com/microsoft/onnxruntime-inference-examples/blob/main/c_cxx/OpenVINO_EP/Linux/squeezenet_classification/squeezenet_cpp_app.cpp

using namespace Eigen;
using namespace Hopper_t;

ZeroDynamicsPolicy::ZeroDynamicsPolicy(std::string model_name) {
    YAML::Node config = YAML::LoadFile("../config/gains.yaml");
    scalar_t stance_time = config["MPC"]["groundDuration"].as<scalar_t>();

    params.kx_p = config["RaibertHeuristic"]["kx_p"].as<scalar_t>();
    params.ky_p = config["RaibertHeuristic"]["ky_p"].as<scalar_t>();
    params.kx_d = config["RaibertHeuristic"]["kx_d"].as<scalar_t>();
    params.ky_d = config["RaibertHeuristic"]["ky_d"].as<scalar_t>();
    params.angle_max = config["RaibertHeuristic"]["angle_max"].as<scalar_t>();
    params.pitch_d_offset = config["RaibertHeuristic"]["pitch_d_offset"].as<scalar_t>();
    params.roll_d_offset = config["RaibertHeuristic"]["roll_d_offset"].as<scalar_t>();

    // onnxruntime setup
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "example-model-explorer");
    Ort::SessionOptions session_options;
    Ort::Session session = Ort::Session(env, model_name.c_str(), session_options);
    size_t numInputNodes = session.GetInputCount();
    size_t numOutputNodes = session.GetOutputCount();
    
    std::cout << "Number of Input Nodes: " << numInputNodes << std::endl;
    std::cout << "Number of Output Nodes: " << numOutputNodes << std::endl;

}



quat_t ZeroDynamicsPolicy::DesiredQuaternion(scalar_t x_a, scalar_t y_a, scalar_t x_d, scalar_t y_d, scalar_t xd_a, scalar_t yd_a, scalar_t yaw_des, vector_3t currentEulerAngles){
	quat_t desQuat(1,0,0,0);
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
