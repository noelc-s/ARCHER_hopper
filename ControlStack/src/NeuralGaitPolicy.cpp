#include "../inc/NeuralGaitPolicy.h"

// #include <algorithm>

// Constructor
NeuralGaitPolicy::NeuralGaitPolicy(const std::string& modelFilepath) {
  /**************** Create ORT environment ******************/
  std::string instanceName{"Image classifier inference"};
  mEnv = std::make_shared<Ort::Env>(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING,
                                    instanceName.c_str());

  /**************** Create ORT session ******************/
  // Set up options for session
  Ort::SessionOptions sessionOptions;
  // Enable CUDA
  sessionOptions.AppendExecutionProvider_CUDA(OrtCUDAProviderOptions{});
  // Sets graph optimization level (Here, enable all possible optimizations)
  sessionOptions.SetGraphOptimizationLevel(
      GraphOptimizationLevel::ORT_ENABLE_ALL);
  // Create session by loading the onnx model
  mSession = std::make_shared<Ort::Session>(*mEnv, modelFilepath.c_str(),
                                            sessionOptions);

  /**************** Create allocator ******************/
  // Allocator is used to get model information
  Ort::AllocatorWithDefaultOptions allocator;

  /**************** Input info ******************/
  // Get the number of input nodes
  size_t numInputNodes = mSession->GetInputCount();

  // Get the name of the input
  // 0 means the first input of the model
  // The example only has one input, so use 0 here
  mInputName = mSession->GetInputName(0, allocator);

  // Get the type of the input
  // 0 means the first input of the model
  Ort::TypeInfo inputTypeInfo = mSession->GetInputTypeInfo(0);
  auto inputTensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
  ONNXTensorElementDataType inputType = inputTensorInfo.GetElementType();

  // Get the shape of the input
  mInputDims = inputTensorInfo.GetShape();

  /**************** Output info ******************/
  // Get the number of output nodes
  size_t numOutputNodes = mSession->GetOutputCount();

  // Get the name of the output
  // 0 means the first output of the model
  // The example only has one output, so use 0 here
  mOutputName = mSession->GetOutputName(0, allocator);

  // Get the type of the output
  // 0 means the first output of the model
  Ort::TypeInfo outputTypeInfo = mSession->GetOutputTypeInfo(0);
  auto outputTensorInfo = outputTypeInfo.GetTensorTypeAndShapeInfo();
  ONNXTensorElementDataType outputType = outputTensorInfo.GetElementType();

  // Get the shape of the output
  mOutputDims = outputTensorInfo.GetShape();
}

// Perform inference for a given image
vector_2t NeuralGaitPolicy::Inference(vector_t states) {
  // Load an input image
//   cv::Mat imageBGR = cv::imread(imageFilepath, cv::IMREAD_COLOR);

  /**************** Preprocessing ******************/
  // Create input tensor (including size and value) from the loaded input image

  // Compute the product of all input dimension
//   size_t inputTensorSize = vectorProduct(mInputDims);
  scalar_t inputTensorSize = 4;
  vector_t inputTensorValues(inputTensorSize);
  // Load the image into the inputTensorValues
//   CreateTensorFromImage(imageBGR, inputTensorValues);
  // input tensor are the follows: x, y, xdot, ydot
  inputTensorValues << states(1), states(2), states(8), states(9);

  // Assign memory for input tensor
  // inputTensors will be used by the Session Run for inference
  std::vector<Ort::Value> inputTensors;
  Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
      OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
  inputTensors.push_back(Ort::Value::CreateTensor<double>(
      memoryInfo, inputTensorValues.data(), inputTensorSize, mInputDims.data(),
      mInputDims.size()));

  // Create output tensor (including size and value)
  size_t outputTensorSize = 2;
  std::vector<double> outputTensorValues(outputTensorSize);

  // Assign memory for output tensors
  // outputTensors will be used by the Session Run for inference
  std::vector<Ort::Value> outputTensors;
  outputTensors.push_back(Ort::Value::CreateTensor<double>(
      memoryInfo, outputTensorValues.data(), outputTensorSize,
      mOutputDims.data(), mOutputDims.size()));


  /**************** Inference ******************/
  // 1 means number of inputs and outputs
  // InputTensors and OutputTensors, and inputNames and
  // outputNames are used in Session Run
  std::vector<const char*> inputNames{mInputName};
  std::vector<const char*> outputNames{mOutputName};
  mSession->Run(Ort::RunOptions{nullptr}, inputNames.data(),
                inputTensors.data(), 1, outputNames.data(),
                outputTensors.data(), 1);


  /**************** Postprocessing the output result ******************/
  // Get the inference result
  const double* tensorData = outputTensors.front().GetTensorData<double>();

// Extract the values and save them in a vector
  vector_2t values(tensorData, tensorData + 2);
//   float* floatarr = outputTensors.front().GetTensorMutableData<double>();
//   // Compute the index of the predicted class
//   // 10 means number of classes in total
//   int cls_idx = std::max_element(floatarr, floatarr + 10) - floatarr;

  return values;
}
