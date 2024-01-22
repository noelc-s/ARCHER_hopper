#include <iostream>
#include "Types.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include "yaml-cpp/yaml.h"
#include <math.h>

using namespace Hopper_t;
using namespace Eigen;

class Policy{
public:
    Policy(){};
    ~Policy(){};        

    template <typename T> int sgn(T val);

    quat_t eulerToQuaternion(scalar_t roll, scalar_t pitch, scalar_t yaw);

    quat_t DesiredQuaternion(scalar_t x_a, scalar_t y_a, scalar_t x_d, scalar_t y_d);
    
    vector_3t DesiredOmega();
    
    vector_4t DesiredInputs();

};