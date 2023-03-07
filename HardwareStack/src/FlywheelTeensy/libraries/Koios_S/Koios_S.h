/*
  Koios_S.h - Library for running the Koios Control Board on ARCHER

  Created by Eric R. Ambrose on July 5th, 2021
  Copyright AMBER Lab, Caltech
*/

#ifndef Koios_S_h
#define Koios_S_h

#include "Arduino.h"
#include "Archer_Config.h"
#include "TripENC.h"
#include "SPI.h"
#include "ELMO_Serial.h"
#include "SD.h"

namespace Archer
{
  class Koios_S
  {
    public:
      Koios_S(TripENC &tENC, ELMO_Serial &elmo1, ELMO_Serial &elmo2, ELMO_Serial &elmo3); //

      void initComm();
      void initSD();
      void setLEDs(String val);
      void setLogo(char Color);
      void setSigB(int val);
      int  checkSwitch();
      void waitSwitch();
      void STO(int val);              // status of 0 = Off, 1 = On
      void resetState(int encID);     // gets new values for the 'prev' T/C
      void updateState(int encID, float &x, float &v);  // gets new values for all T/C states
      void track(int encID, float xd, float vd);
      void comp(float d);
      void delayLoop(uint32_t T0, uint32_t dTdes);
    private:
      TripENC &tENC_;
      ELMO_Serial &elmo1_;
      ELMO_Serial &elmo2_;
      ELMO_Serial &elmo3_;
      int _sOUT   = 31;
      int _sIN    = 32;
      int _relayK = 33;
      int _relayR = 34;
      int _relayA = 35;
      int _relayG = 36;
      int _sigB   = 2;
      int _R      = 40;
      int _Y1     = 39;
      int _Y2     = 38;
      int _G      = 37;

      uint32_t _prevT1;
      uint32_t _nextT1;
      long     _prevC1;
      long     _nextC1;
      uint32_t _prevT2;
      uint32_t _nextT2;
      long     _prevC2;
      long     _nextC2;
      uint32_t _prevT3;
      uint32_t _nextT3;
      long     _prevC3;
      long     _nextC3;

      float _resK = 102400.0; // Counts per revolution
      float _x1;
      float _v1;
      float _x2;
      float _v2;
      float _x3;
      float _v3;
      float _KpK = 3;
      float _KdK = 1;
  };
}

#endif