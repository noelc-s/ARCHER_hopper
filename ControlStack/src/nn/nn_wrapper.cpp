#include "../../inc/nn/nn_interface.h"
#include <onnxruntime_cxx_api.h>

class NNDummy : public NNInterface
{
public:
    Ort::Env env;
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator;
    std::string inputNodeName;
    std::string outputNodeName;
    std::unique_ptr<Ort::TypeInfo> inputTypeInfo;
    ONNXTensorElementDataType inputType;
    std::vector<int64_t> inputDims;
    size_t inputTensorSize;
    std::unique_ptr<Ort::TypeInfo> outputTypeInfo;
    ONNXTensorElementDataType outputType;
    std::vector<int64_t> outputDims;
    size_t outputTensorSize;   

    NNWrapper(const std::string model_name) {
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

    void evaluateNetwork(const vector_t& input, vector_t& ouptut) override
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
};

std::unique_ptr<NNInterface> createNNInstance(const std::string model_name)
{
    return std::make_unique<NNWrapper>(model_name);
}