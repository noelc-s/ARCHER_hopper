/*
  Bia_S.h - Library for running the Bia Control Board on ARCHER

  Created by Eric R. Ambrose on July 2nd, 2021
  Copyright AMBER Lab, Caltech
*/

#ifndef Bia_S_h
#define Bia_S_h

#include "Archer_Config.h"
#include "Arduino.h"
#include "DualENC.h"
#include "SPI.h"
#include "ELMO_Serial.h"
#include "SD.h"

namespace Archer
{
  class Bia_S
  {
    public:
      Bia_S(DualENC &dENC, ELMO_Serial &elmo); //

      void initComm();
      void initSD();
      void setLEDs(String val);
      int  checkSigK();
      void waitSigK(int trigger);
      void STO(int val);              // status of 0 = Off, 1 = On
      void resetState(int encID);     // gets new values for the 'prev' T/C
      void updateState(int encID, float &x, float &v);  // gets new values for all T/C states
      void track(float xd, float vd, int load);
      void comp(float d);
      void delayLoop(uint32_t T0, uint32_t dTdes);
    private:
      DualENC &dENC_;
      ELMO_Serial &elmo_;
      int _relayB   = 33;
      int _sigK     = 6;
      int _encID_B  = 1;
      int _encID_F  = 2;
      int _R        = 34;
      int _Y1       = 35;
      int _Y2       = 36;
      int _G        = 37;

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
      float _resF = 1024000.0; // Counts per revolution
      float _xF;
      float _vF;
      float _KpB = 1;
      float _KdB = 1;
  };
}

#endif