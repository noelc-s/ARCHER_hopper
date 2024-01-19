#include "../inc/Policy.h"


quat_t Policy::DesiredQuaternion(){
   
    quat_t quat_des = Eigen::Quaternion<scalar_t>(1,0,0,0);

    return quat_des;

}


vector_3t Policy::DesiredOmega(){
   
    vector_3t omega_des;
    omega_des << 0, 0, 0;
    
    return omega_des;

}

vector_4t Policy::DesiredInputs(){

    vector_4t u_des;
    u_des << 0, 0, 0, 0;
    
    return u_des;
}