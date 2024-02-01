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

    // onnxruntime setup
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "example-model-explorer");
    Ort::SessionOptions session_options;
    Ort::Session session = Ort::Session(env, model_name.c_str(), session_options);
    Ort::AllocatorWithDefaultOptions allocator;
    size_t numInputNodes = session.GetInputCount();
    size_t numOutputNodes = session.GetOutputCount();
    
    std::cout << "Number of Input Nodes: " << numInputNodes << std::endl;
    std::cout << "Number of Output Nodes: " << numOutputNodes << std::endl;

    auto inputNodeName = session.GetInputNameAllocated(0, allocator);
    const char* inputName = inputNodeName.get();
    std::cout << "Input Name: " << inputName << std::endl;

     Ort::TypeInfo inputTypeInfo = session.GetInputTypeInfo(0);
    auto inputTensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();

    ONNXTensorElementDataType inputType = inputTensorInfo.GetElementType();
    std::cout << "Input Type: " << inputType << std::endl;

    std::vector<int64_t> inputDims = inputTensorInfo.GetShape();
    for (const auto &e : inputDims)
	std::cout << "Input Dimensions: " << e << std::endl;

    auto outputNodeName = session.GetOutputNameAllocated(0, allocator);
    const char* outputName = outputNodeName.get();
    std::cout << "Output Name: " << outputName << std::endl;

    Ort::TypeInfo outputTypeInfo = session.GetOutputTypeInfo(0);
    auto outputTensorInfo = outputTypeInfo.GetTensorTypeAndShapeInfo();

    ONNXTensorElementDataType outputType = outputTensorInfo.GetElementType();
    std::cout << "Output Type: " << outputType << std::endl;

    std::vector<int64_t> outputDims = outputTensorInfo.GetShape();
    for (const auto &e : outputDims)
    std::cout << "Output Dimensions: " << e << std::endl;

    
    size_t inputTensorSize = vectorProduct(inputDims);
    size_t outputTensorSize = vectorProduct(outputDims);

    std::vector<const char*> inputNames{inputName};
    std::vector<const char*> outputNames{outputName};
    std::vector<Ort::Value> inputTensors;
    std::vector<Ort::Value> outputTensors;

 
    std::vector<float> input(4);
    input[0] = 1;
    input[1] = 1;
    input[2] = 1;
    input[3] = 1;

    std::vector<float> output(2);



    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    inputTensors.push_back(Ort::Value::CreateTensor<float>(
        memoryInfo, input.data(), inputTensorSize, inputDims.data(),
        inputDims.size()));
    outputTensors.push_back(Ort::Value::CreateTensor<float>(
        memoryInfo, output.data(), outputTensorSize,
        outputDims.data(), outputDims.size()));
    session.Run(Ort::RunOptions{nullptr}, inputNames.data(),
                inputTensors.data(), 1, outputNames.data(),
                outputTensors.data(), 1);
    for (const auto &e : output)
	    std::cout << e << std::endl;


}

void ZeroDynamicsPolicy::EvaluateNetwork(const vector_4t state, vector_2t& output) {
    
    std::vector<float> input(4);
    input[0] = state(0);
    input[1] = state(1);
    input[2] = state(2);
    input[3] = state(3);

    std::vector<float> outpt(2);

    //session.Run(Ort::RunOptions{nullptr}, inputNames.data(),
    //            inputTensors.data(), 1, outputNames.data(),
    //            outputTensors.data(), 1);
    output << output[0], output[1];
}

quat_t ZeroDynamicsPolicy::DesiredQuaternion(scalar_t x_a, scalar_t y_a, scalar_t x_d, scalar_t y_d, scalar_t xd_a, scalar_t yd_a, scalar_t yaw_des, vector_3t currentEulerAngles){
	quat_t desQuat(1,0,0,0);

	vector_4t state;
	state << x_a - x_d, y_a - y_d, xd_a, yd_a; 
	vector_2t rp_des;
	EvaluateNetwork(state, rp_des);

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
