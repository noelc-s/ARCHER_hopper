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

    /*! @brief  evaluate the forward dynamics
    *  @param [in] val  signed variable
    *  @param [out] sign sign of the variable (+/-1)
    */
    template <typename T> int sgn(T val);

    /*! @brief  evaluate the forward dynamics
    *  @param [in] roll  roll angle of the body frame wrt the world frame
    *  @param [in] pitch  pitch angle of the body frame wrt the world frame
    *  @param [in] yaw  yaw angle of the body frame wrt the world frame
    *  @param [out] quaternion  quaternion representation of the orientation
    */
    quat_t eulerToQuaternion(scalar_t roll, scalar_t pitch, scalar_t yaw);

    /*! @brief  evaluate the forward dynamics
    *  @param [in] x_a  current position (x-direction) of the body frame wrt the world frame
    *  @param [in] y_a  current position (y-direction) of the body frame wrt the world frame
    *  @param [in] x_d  desired position (x-direction) of the body frame wrt the world frame
    *  @param [in] y_d  desired position (y-direction) of the body frame wrt the world frame
    *  @param [out] quat_d  desired quaternion for the low level controller
    */
    quat_t DesiredQuaternion(scalar_t x_a, scalar_t y_a, scalar_t x_d, scalar_t y_d);
    
    /*! @brief  evaluate the forward dynamics
    *  @param [out] omega_d  desired omega (rate of change of quaternion) of the body frame wrt the world fram
    */
    vector_3t DesiredOmega();
    
    /*! @brief  evaluate the forward dynamics
    *  @param [out] u_des  desired feedforward input
    */
    vector_4t DesiredInputs();

};