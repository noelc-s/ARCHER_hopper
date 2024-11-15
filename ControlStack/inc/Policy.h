#include <iostream>
#include "Types.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include "yaml-cpp/yaml.h"
#include <math.h>

#include "Hopper.h"
#include "utils.h"
// #include "MPC.h"
// #include <onnxruntime_cxx_api.h>
#include <numeric>

using namespace Hopper_t;
using namespace Eigen;

class Policy{
public:
    virtual ~Policy(){}; 

    struct Params {
      scalar_t kx_p;
      scalar_t ky_p;
      scalar_t kx_d;
      scalar_t ky_d;
      scalar_t kx_f;
      scalar_t ky_f;
      scalar_t p_clip;
      scalar_t v_clip;
      scalar_t vd_clip;
      scalar_t angle_max;
      scalar_t pitch_d_offset;
      scalar_t roll_d_offset;
      scalar_t yaw_damping;
    } params;

    void loadParams(std::string filepath, Params& params);   


    static vector_3t Quaternion2Euler(const quat_t& q);
    
    /*! @brief  evaluate the forward dynamics
    *  @param [in] x_a  current position (x-direction) of the body frame wrt the world frame
    *  @param [in] y_a  current position (y-direction) of the body frame wrt the world frame
    *  @param [in] x_d  desired position (x-direction) of the body frame wrt the world frame
    *  @param [in] y_d  desired position (y-direction) of the body frame wrt the world frame
    *  @param [in] contact  boolean of whether the robot is in contact with the ground
    *  @param [out] quat_d  desired quaternion for the low level controller
    */
    virtual quat_t DesiredQuaternion(Hopper::State state, matrix_t command) = 0;
    
    /*! @brief  evaluate the forward dynamics
    *  @param [out] omega_d  desired omega (rate of change of quaternion) of the body frame wrt the world fram
    */
    virtual vector_3t DesiredOmega() = 0;
    
    /*! @brief  evaluate the forward dynamics
    *  @param [out] u_des  desired feedforward input
    */
    virtual vector_4t DesiredInputs(const vector_3t wheel_vel, const bool contact) = 0;
};

class RaibertPolicy : public Policy{
public:
    RaibertPolicy(const std::string yamlPath);
    quat_t DesiredQuaternion(Hopper::State state, matrix_t command);
    vector_3t DesiredOmega();
    vector_4t DesiredInputs(const vector_3t wheel_vel, const bool contact);
};

// Based on:
// https://github.com/microsoft/onnxruntime-inference-examples/blob/main/c_cxx/OpenVINO_EP/Linux/squeezenet_classification/squeezenet_cpp_app.cpp
// class ZeroDynamicsPolicy : public Policy{
// public:
//     ZeroDynamicsPolicy(std::string model_name, const std::string yamlPath);
//     void EvaluateNetwork(const vector_4t state, vector_2t& output);
//     quat_t DesiredQuaternion(Hopper::State state, matrix_t command);
//     vector_3t DesiredOmega();
//     vector_4t DesiredInputs(const vector_3t wheel_vel, const bool contact);

//     Ort::Env env;
//     std::unique_ptr<Ort::Session> session;
//     Ort::AllocatorWithDefaultOptions allocator;
//     std::string inputNodeName;
//     std::string outputNodeName;
//     std::unique_ptr<Ort::TypeInfo> inputTypeInfo;
//     ONNXTensorElementDataType inputType;
//     std::vector<int64_t> inputDims;
//     size_t inputTensorSize;
//     std::unique_ptr<Ort::TypeInfo> outputTypeInfo;
//     ONNXTensorElementDataType outputType;
//     std::vector<int64_t> outputDims;
//     size_t outputTensorSize;    
// };

// class MPCPolicy : public Policy {
// public:
//     MPCPolicy(const std::string yamlPath, std::shared_ptr<Hopper> hopper, std::shared_ptr<MPC> mpc);

//     std::shared_ptr<Hopper> hopper;
//     std::shared_ptr<MPC> mpc_;
//     vector_t q0, q0_local;
//     vector_t sol, sol_g;
//     matrix_t x_pred, u_pred;
//     scalar_t dt_elapsed_MPC, t_last_MPC;
    

//     quat_t DesiredQuaternion(Hopper::State state, matrix_t command);
//     vector_3t DesiredOmega();
//     vector_4t DesiredInputs(const vector_3t wheel_vel, const bool contact);
// };

// class RLPolicy : public Policy {
// public:
//     RLPolicy(std::string model_name, const std::string yamlPath);
//     void EvaluateNetwork(const Hopper::State state, const matrix_t command, vector_4t& output);
//     quat_t DesiredQuaternion(Hopper::State state, matrix_t command);
//     vector_3t DesiredOmega();
//     vector_4t DesiredInputs(const vector_3t wheel_vel, const bool contact);

//     vector_4t q_des;
//     scalar_t dt_elapsed_RL;
//     scalar_t t_last_RL;

//     vector_4t previous_action;
//     Ort::Env env;
//     std::unique_ptr<Ort::Session> session;
//     Ort::AllocatorWithDefaultOptions allocator;
//     std::string inputNodeName;
//     std::string outputNodeName;
//     std::unique_ptr<Ort::TypeInfo> inputTypeInfo;
//     ONNXTensorElementDataType inputType;
//     std::vector<int64_t> inputDims;
//     size_t inputTensorSize;
//     std::unique_ptr<Ort::TypeInfo> outputTypeInfo;
//     ONNXTensorElementDataType outputType;
//     std::vector<int64_t> outputDims;
//     size_t outputTensorSize;    

//     struct RLParams {
//       scalar_t lin_vel_scaling;
//       scalar_t ang_vel_scaling;
//       scalar_t dof_vel_scaling;
//       scalar_t z_pos_scaling;
//       scalar_t dt_replan;
//     } RLparams;

//     void loadParams(std::string filepath, RLParams& params);  
// };


// class RLTrajPolicy : public Policy {
// public:
//     RLTrajPolicy(std::string model_name, const std::string yamlPath, int horizon, int state_dim);
//     void EvaluateNetwork(const Hopper::State state, const matrix_t command, vector_4t& output);
//     quat_t DesiredQuaternion(Hopper::State state, matrix_t command);
//     vector_3t DesiredOmega();
//     vector_4t DesiredInputs(const vector_3t wheel_vel, const bool contact);

//     vector_4t q_des;
//     scalar_t dt_elapsed_RL;
//     scalar_t t_last_RL;
//     const int horizon;
//     const int state_dim;

//     vector_4t previous_action;
//     Ort::Env env;
//     std::unique_ptr<Ort::Session> session;
//     Ort::AllocatorWithDefaultOptions allocator;
//     std::string inputNodeName;
//     std::string outputNodeName;
//     std::unique_ptr<Ort::TypeInfo> inputTypeInfo;
//     ONNXTensorElementDataType inputType;
//     std::vector<int64_t> inputDims;
//     size_t inputTensorSize;
//     std::unique_ptr<Ort::TypeInfo> outputTypeInfo;
//     ONNXTensorElementDataType outputType;
//     std::vector<int64_t> outputDims;
//     size_t outputTensorSize;    

//     struct RLParams {
//       scalar_t lin_vel_scaling;
//       scalar_t ang_vel_scaling;
//       scalar_t dof_vel_scaling;
//       scalar_t z_pos_scaling;
//       scalar_t dt_replan;
//     } RLparams;

//     void loadParams(std::string filepath, RLParams& params);  
// };