/*
  Main_TBLin.h - Library for functional control of the Linear motor testbed via the Main_Twitter PCB

  Created by Eric R. Ambrose on January 15th, 2020
  Copyright AMBER Lab, Caltech
*/
#ifndef Main_TBLin_h
#define Main_TBLin_h

#include "Arduino.h"
#include "DualENC.h"
#include "Twitter.h"
#include "PCBpixar.h"
#include "SPI.h"
#include "SD.h"

class Main_TBLin
{
  public:
    Main_TBLin(DualENC &dENC,Twitter &elmo,PCBpixar &pcb);
    void resetState();                                                // Reset state time
    void updateState(float &x, float&v);                              // Update states to latest value
    void delayLoop(uint32_t T0, uint32_t dtDes);                      // Delays loop to given duration
    void motorStartup();                                              // Wait for STO and start motor
    void sendCmd(float desI, float Imax);                             // Saturate and send TC command
    void desTraj(float t, float &yDes, float &dyDes, float &ddyDes);  // Calc desired trajectory states
    void initPos(float &x, float &v);                                 // Go to initial position
    void trackTraj(uint32_t T0);                                      // Track desired traj in CL
    void stopMotion();                                                // Motor off and prepare for startup
  private:
    DualENC  &dENC_;
    Twitter  &elmo_;
    PCBpixar &pcb_;

    uint32_t _prevT1;
    uint32_t _nextT1;
    long     _prevC1;
    long     _nextC1;

    float _res1 = 1024000.0;
    float _x1;
    float _v1;
    float _M  = 1.865;
    float _kf = 11.3;
};
#endif