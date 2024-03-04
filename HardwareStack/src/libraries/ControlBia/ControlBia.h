/*
  ControlBia.h - Library for the Bia Controller Calculations on ARCHER

  Created by Eric R. Ambrose on Sep 29th, 2021
  Copyright AMBER Lab, Caltech
*/

#ifndef ControlBia_h
#define ControlBia_h

#include "Arduino.h"
#include "Archer_Config.h"

namespace Archer
{
  class ControlBia
  {
    public:
      ControlBia(float Tmax); //

      void resetI();
      void getRB0(float &val);
      void inputPID0(float &u,float d0,float rb,float wb,float xf);
      void testPID0(float &u0,float &up,float &ud,float &ui,float d0,float rb,float wb);
      void testPDb(float &up,float &ud,float rb, float wb);
      void testPDf(float &u0,float &up,float &ud,float d0,float xf, float vf);
      void testU0(float &up,float &ud,float d0,float xf, float vf);
      void releaseFoot(float &up,float &ud,float rb, float wb);
      void logZero(float val,int i);
      void saturate(float &u);
      void setTx(int val);
    private:
      float kP  = 200.0;     // 
      float kI  = 0.00;      // 
      float kD  = 2.00;      // 
      float kPb = 2.50;      // 
      float kDb = 0.1;     // 
      float _Tmax;           // Max current allowed
      float _Imax = 2.0;     // Max current allowed from controller's I term
      float _rad  = 0.0375;  // Pulley radius
      float _gr   = 3.0;     // Bia gear ratio
      float _ks   = 6213.0;  // Spring constant N/m
      float _kf   = 0.185;   // Motor constant Nm/A
      float _st   = 1.425;   // Cable stretch factor, roughly for current range

      float _I;
      float _rb0;
      float _rb1;
      float _rb2;
  };
}

#endif