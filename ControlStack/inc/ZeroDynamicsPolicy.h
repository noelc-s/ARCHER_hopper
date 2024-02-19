#include <iostream>
#include "Types.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include "yaml-cpp/yaml.h"
#include <math.h>
#include <onnxruntime_cxx_api.h>
#include <numeric>

using namespace Hopper_t;
using namespace Eigen;

class ZeroDynamicsPolicy{
public:
    ZeroDynamicsPolicy(std::string model_name);
    struct Params {
      scalar_t kx_p;
      scalar_t ky_p;
      scalar_t kx_d;
      scalar_t ky_d;
      scalar_t angle_max;
      scalar_t pitch_d_offset;
      scalar_t roll_d_offset;
      scalar_t yaw_damping;
    } params;

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

    void EvaluateNetwork(const vector_4t state, vector_2t& output);

    quat_t DesiredQuaternion(scalar_t x_a, scalar_t y_a, scalar_t x_d, scalar_t y_d, scalar_t xd_a, scalar_t yd_a, scalar_t yaw_des, vector_3t currentEulerAngles);
    vector_3t DesiredOmega();
    vector_4t DesiredInputs();
};
