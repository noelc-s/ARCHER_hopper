#include<iostream>
#include <eigen3/Eigen/Dense>
// #include "../inc/Types.h"
#include<vector>

// using namespace Hopper_t;

using namespace Eigen;

Quaternionf Euler2Quaternion(float roll, float pitch, float yaw){
    Quaternionf q = AngleAxisf(roll, Vector3f::UnitX())
                    * AngleAxisf(pitch, Vector3f::UnitY())
                    * AngleAxisf(yaw, Vector3f::UnitZ());
    
    return q;
                       
}

Quaternionf MultiplyQuaternions(Quaternionf input, Quaternionf multiplier){
    Quaternionf q;

    q = (multiplier)*(input)*(multiplier.inverse());

    return q;
}


Quaternionf YawTransformation(Vector3f currentEulerAngles, Vector3f desiredEulerAngles){
    // EulerAngles == roll, pitch, yaw 

    // generating the quaternion map from current yaw only for mapping back and forth
    Quaternionf currentYawQuaternionInverse;
    currentYawQuaternionInverse = Euler2Quaternion(0, 0, currentEulerAngles[2]).inverse();
    // Quaternionf currentYawQuaternionInverse = currentYawQuaternion.inverse();

    // converting the desired euler angles for quaternion multiplication [0, roll, pitch, yaw]
    Quaternionf desiredEulerAnglesQuatRep;
    desiredEulerAnglesQuatRep.w() = 0;
    desiredEulerAnglesQuatRep.vec() = desiredEulerAngles;

    // changing to local basis
    Quaternionf inputLocalBasis = MultiplyQuaternions(desiredEulerAnglesQuatRep, currentYawQuaternionInverse);
    
    return inputLocalBasis;

}
int main(){

    // current euler orientation in the global frame 
    float roll_a = 0.2;
    float pitch_a = 0.4;
    float yaw_a = 3.14/2;

    Vector3f CurrentEulerAngles;
    CurrentEulerAngles << roll_a, pitch_a, yaw_a;
    std::cout<<"Current Global Orientation"<< std::endl << CurrentEulerAngles << std::endl;
    // std::cout<<"Current global orientation"<<std::endl<<roll_a<<std::endl<<pitch_a<<std::endl<<yaw_a<<std::endl;
    Quaternionf q;
    
    // current quaternion orientation in the global frame
    q  = Euler2Quaternion(roll_a, pitch_a, yaw_a);
    
    // std::cout << "Quaternion coefficients from euler angles" << std::endl << q.coeffs() << std::endl;

    // Eigen::Quaternion<float> test_quaternion = Eigen::Quaternion<float>(0.5,0.1,0.4,0);
    Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    // std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler << std::endl;
    // std::cout<<"Initial yaw: "<<yaw_a<<std::endl;
    // std::cout<<" Final yaw: "<<euler[2]<<std::endl;
    
    // building the quaternion and inverse quaternion purely from current yaw
    Quaternionf yaw_q;
    yaw_q = Euler2Quaternion(0, 0, euler[2]);
    
    Quaternionf yaw_q_inv = yaw_q.inverse();
    // std::cout<<"Quaternion from purely yaw: "<< std::endl << yaw_q.coeffs() <<std::endl;
    // std::cout<<"Inverse quaternion from purely yaw: "<< std::endl << yaw_q_inv.coeffs() <<std::endl;
    
    // desired euler angles from the heuristic
    float pitch_d = 0.0;
    float roll_d = 0.6;
    float yaw_d = 0;

    // packing the in a vector for easier printing, pls ignore 
    Vector3f desiredEulerAngles;
    desiredEulerAngles << roll_d, pitch_d, yaw_d;

    std::cout<<"Desired Global Euler Angles"<< std::endl << desiredEulerAngles<<std::endl;

    // converting the desired angles into "quaternion form factor" for multiplication
    Quaternionf q_input_global;
    q_input_global.w() = 0;
    q_input_global.vec() = desiredEulerAngles;

    //transformation to local coordinates by multiplying it by yaw inverse
    Quaternionf q_input_local = MultiplyQuaternions(q_input_global, yaw_q_inv);
    Vector3f input_local = q_input_local.vec();

    q_input_local = YawTransformation(CurrentEulerAngles, desiredEulerAngles);

    std::cout<<"Desired Local Euler Angles"<< std::endl << input_local<<std::endl;

    //extracting the desired angles and changing it to euler coordinates
    Quaternionf q_desired_angles = Euler2Quaternion(input_local(0), input_local(1), input_local(2));
    //transforming back to global coordinates by multiplying it by yaw_q

    Quaternionf q_desired_angles_transformed = MultiplyQuaternions(q_desired_angles, yaw_q);

    std::cout<<"Desired Quaternion Angle" << std::endl << q_desired_angles_transformed.coeffs()<<std::endl;

    Vector3f desiredGlobalAngle = q_desired_angles_transformed.toRotationMatrix().eulerAngles(0, 1, 2);

    std::cout<<"What is this then "<< std::endl<< desiredGlobalAngle<<std::endl;
    return 0;
}