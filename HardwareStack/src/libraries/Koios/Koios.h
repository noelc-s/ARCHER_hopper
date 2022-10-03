/*
  Koios.h - Library for running the Koios Control Board on ARCHER via CAN
            Final Experiment Configuration

  Created by Eric R. Ambrose on March 27th, 2022
  Copyright AMBER Lab, Caltech
*/

#ifndef Koios_h
#define Koios_h

#include "Arduino.h"
#include "Archer_Config.h"
#include "TripENC.h"
#include "SPI.h"
#include "ELMO_CANt4.h"
#include "FlexCAN_T4.h"
#include "SD.h"

namespace Archer
{
  class Koios
  {
    public:
      Koios(TripENC &tENC, ELMO_CANt4 &elmo); //

      int32_t initComm(int MC);
      void initSD();
      void initIMU();
      void initKoios1(int MC);
      void initKoios2();
      int getIntFromB();
      void setLEDs(String val);
      void flashR(int Rep);
      void flashA1(int Rep);
      void flashA2(int Rep);
      void flashG(int Rep);
      void setLogo(char Color);
      void setSigB(int val);
      void reverseSig(int val);
      int  checkSigB();
      int  checkSwitch();
      void waitSwitch(int val);
      void STO(int val);              // status of 0 = Off, 1 = On
      int32_t motorsOff(int val);     // Sends all 3 motorOff commands, then sets STO to 'val'
      int32_t motorsOn();             // Sends all 3 motorOn commands
      void resetStates();             // Sets new values for the "prev" states/times
      void updateStates(volatile float &x1,volatile float &v1,volatile float &x2,volatile float &v2,volatile float &x3,volatile float &v3);
      int checkFrame(float q0,float q1,float q2,float q3,float dY,float dP,float dR);
      int updateAtt(float &Y,float &P,float &R);
      void delayLoop(uint32_t T0, uint32_t dTdes);
    private:
      TripENC     &tENC_;
      ELMO_CANt4  &elmo_;
      int32_t     rt = 0;

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

      float _q0;
      float _q1;
      float _q2;
      float _q3;
      float _Yaw;
      float _Pitch;
      float _Roll;
      float _dY;
      float _dP;
      float _dR;
  };
}

#endif
