#include "../inc/ZeroDynamicsPolicy.h"

// Based on:
// https://github.com/microsoft/onnxruntime-inference-examples/blob/main/c_cxx/OpenVINO_EP/Linux/squeezenet_classification/squeezenet_cpp_app.cpp

using namespace Eigen;
using namespace Hopper_t;

template <typename T>
T vectorProduct(const std::vector<T>& v)
{
    return std::accumulate(v.begin(), v.end(), 1, std::multiplies<T>());
}

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

quat_t ZeroDynamicsPolicy::DesiredQuaternion(scalar_t x_a, scalar_t y_a, scalar_t x_d, scalar_t y_d, scalar_t xd_a, scalar_t yd_a, scalar_t yaw_des, vector_3t currentEulerAngles){

	vector_4t state;
	state << x_a - x_d, xd_a, y_a - y_d,  yd_a; 
	vector_2t rp_des;
	EvaluateNetwork(state, rp_des);

	quat_t desQuat = AngleAxisd(rp_des(1), Vector3d::UnitX())
                    * AngleAxisd(rp_des(0), Vector3d::UnitY())
                    * AngleAxisd(yaw_des, Vector3d::UnitZ());

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
