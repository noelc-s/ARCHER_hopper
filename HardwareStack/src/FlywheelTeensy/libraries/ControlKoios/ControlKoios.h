/*
  ControlKoios.h - Library for the Koios Controller Calculations on ARCHER

  Created by Eric R. Ambrose on Sep 11th, 2021
  Copyright AMBER Lab, Caltech
*/

#ifndef ControlKoios_h
#define ControlKoios_h

#include "Arduino.h"
#include "Archer_Config.h"

namespace Archer
{
  class ControlKoios
  {
    public:
      ControlKoios(float Tmax); //

      void inputs(float &u1,float &u2,float &u3,float Y,float P,float R,float dY,float dP,float dR);
      void rotZYX(float &V1,float &V2,float &V3);
      void euler2quat(float &Q0,float &Q1,float &Q2,float &Q3);
      void quatMult(float &Q0,float &Q1,float &Q2,float &Q3);
      void quatAxis(float &V1,float &V2,float &V3);
      float normK(float v1,float v2,float v3);
      float dotK(float v1,float v2,float v3,float w1,float w2,float w3);
      void saturate(float &u1,float &u2,float &u3);
      void filterVel(float vY,float vP,float vR);
      void setKp(int val);
      void setKd(int val);
      void setTx(int val);
      void setYY(int val);
      void setDY(int val);
      void setPP(int val);
      void setDP(int val);
      void setRR(int val);
      void setDR(int val);
    private:
      float Y_des = 0.0;
      float P_des = 0.0;
      float R_des = 0.0;
      float theta = 0.6154797086703873; // 0.5*acos(1/3) in radians
      float kp = 0.0;  // -400 in MATLAB, was -2000.0
      float kd = 0.0;    // 25 in MATLAB, was +400.0
      float YY = 0.0;   // modulating the Yaw state data
      float DY = 0.0;   // modulating the Yaw rate data
      float PP = 0.0;   // modulating the Pitch state data
      float DP = 0.0;   // modulating the Pitch rate data
      float RR = 0.0;   // modulating the Roll state data
      float DR = 0.0;   // modulating the Roll rate data
      float _Tmax;

      float _Yaw;  // States
      float _Pitch;
      float _Roll;
      float _dY = 0.0;
      float _dP = 0.0;
      float _dR = 0.0;
      float _fy = 0.10;
      float _fp = 0.04;
      float _fr = 0.07;

      float _v1; // inputs for vector-based functions
      float _v2;
      float _v3;

      float _e1; // inputs for euler-based funcitons
      float _e2;
      float _e3;

      float _p0; // inputs for quaternion-based functions
      float _p1;
      float _p2;
      float _p3;
      float _q0;
      float _q1;
      float _q2;
      float _q3;
  };
}

#endif