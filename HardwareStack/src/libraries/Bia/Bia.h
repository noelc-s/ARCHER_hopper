/*
  Bia.h - Library for running the Bia Control Board on ARCHER via CAN
          Final Experiment Configuration

  Created by Eric R. Ambrose on March 27th, 2022
  Copyright AMBER Lab, Caltech
*/

#ifndef Bia_h
#define Bia_h

#include "Arduino.h"
#include "Archer_Config.h"
#include "DualENC.h"
#include "SPI.h"
#include "ELMO_CANt4.h"
#include "ControlBia.h"
#include "FlexCAN_T4.h"
#include "SD.h"

namespace Archer
{
  class Bia
  {
    public:
      Bia(DualENC &dENC, ELMO_CANt4 &elmo, ControlBia &cBia); //

      void exitProgram();
      void runSin();
      int32_t sendSafeTorque(float xb, float u);
      void releaseMotor();

      int32_t initComm(int MC);
      void initSD();
      void initBia1(int MC);
      void setLEDs(String val);
      void flashR(int Rep);
      void flashA1(int Rep);
      void flashA2(int Rep);
      void flashG(int Rep);
      int  checkSigK();
      void sendIntToK(int send_val);
      void sendCharArrToK(char* send_chars);
      void reverseSig(int val);
      void setSigK(int val);
      void waitSigK(int trigger);
      void STO(int val);              // status of 0 = Off, 1 = On
      void resetState(int encID);     // gets new values for the 'prev' T/C
      void updateState(int encID, float &x, float &v);  // gets new values for all T/C states
      void trackPID0(float d0,float &x,float &u0);
      void trackU0(float d0,float u0,float &xf,float &vf,float &u);
      void testPID0(float d0,float &x,float &u0,float &up,float &ud,float &ui);
      void testPDb(float &rb,float &wb,float &xf,float &vf,float &up,float &ud);
      void testPDf(float d0,float &rb,float &xf,float &u0,float &up,float &ud);
      void testU0(float d0,float &xf,float u0);
      void updateRB0();
      void delayLoop(uint32_t T0, uint32_t dTdes);
      void setRB0(float val);
    private:
      DualENC      &dENC_;
      ELMO_CANt4   &elmo_;
      ControlBia   &cBia_;
      int32_t rt = 0;

      int _relayB   = 33;
      int _sigK     = 6;
      int _encID_B  = 1;
      int _encID_F  = 2;
      int _R        = 34;
      int _Y1       = 35;
      int _Y2       = 36;
      int _G        = 37;

      double theta_max = 1.2;
      double theta_min = -5.0;

      uint32_t _prevTb;
      uint32_t _nextTb;
      long     _prevCb;
      long     _nextCb;
      uint32_t _prevTf;
      uint32_t _nextTf;
      long     _prevCf;
      long     _nextCf;

      float _resB = 102400.0; // Counts per revolution
      float _x;
      float _v;
      float _resF = 512.0; // Counts per mm
      float _xF;
      float _vF;
      float _KpB = 1;
      float _KdB = 1;
      float _rb0;
  };
}

#endif
