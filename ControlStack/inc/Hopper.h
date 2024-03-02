//
// Created by igor on 7/20/22.
//

#ifndef HOPPER_HOPPER_H
#define HOPPER_HOPPER_H

#include <iostream>
#include "Types.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include "yaml-cpp/yaml.h"
#include "Policy.h"
#include <onnxruntime_cxx_api.h>

using namespace Hopper_t;

enum domain {flight, ground, flight_ground, ground_flight};

class Hopper {

public:
    scalar_t t;
    vector_3t pos;
    quat_t quat;
    vector_3t vel;
    vector_3t omega;
    scalar_t contact;
    scalar_t last_impact_time;
    scalar_t last_flight_time;
    scalar_t leg_pos;
    scalar_t leg_vel;
    vector_3t wheel_vel;
    vector_t q;
    vector_t v;
    domain dom;
    scalar_t multiplier_on_deltaf;

    vector_4t torque;

    quat_t quat_actuator;

    struct Gains {
        vector_3t orientation_kp;
        vector_3t orientation_kd;
        scalar_t leg_kp;
        scalar_t leg_kd;
    } gains;

    Hopper(const std::string yamlFile);

    static matrix_3t cross(vector_3t q);
    static matrix_3t quat2Rot(quat_t q);

    void updateState(vector_t state);

    /*! @brief compute the torque (u_ff + u_feedback)
     * @param [in] quat_d_ desried quaternion
     * @param [in] omega_d desried angular rate
     * @param [in] length_des desried spring compression length
     * @param [in] u_des feed forward torque
     */
    void computeTorque(quat_t quat_d_, vector_3t omega_d, scalar_t length_des, vector_t u_des);

};

class NNHopper : public Hopper {
public:    
    NNHopper(std::string model_name, const std::string yamlPath);
    void EvaluateNetwork(const quat_t quat_err, const vector_3t omega, const vector_3t flywheel_speed, vector_3t& tau);
    void computeTorque(quat_t quat_d_, vector_3t omega_d, scalar_t length_des, vector_t u_des);

    // TODO: consoloidate this with policy class
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

};

#endif //HOPPER_HOPPER_H
