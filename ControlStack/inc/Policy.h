#include <iostream>
#include "Types.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include "yaml-cpp/yaml.h"

using namespace Hopper_t;

class Policy{
public:
    Policy(){};
    ~Policy(){};        
    quat_t DesiredQuaternion();
    
    vector_3t DesiredOmega();
    
    vector_4t DesiredInputs();

};