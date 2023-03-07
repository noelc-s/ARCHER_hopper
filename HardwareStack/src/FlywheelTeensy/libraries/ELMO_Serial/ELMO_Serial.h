/*
  ELMO_Serial.h - Library for sending commands to the ELMO twitter via Serial. This version
                  assumes current command mode (UM=1)

  Created by Eric R. Ambrose on July 2nd, 2021
  Copyright AMBER Lab, Caltech
*/

#ifndef ELMO_Serial_h
#define ELMO_Serial_h

#include "Arduino.h"
#include "Archer_Config.h"

namespace Archer
{
  class ELMO_Serial
  {
    public:
      ELMO_Serial(HardwareSerial &port);            // Init port variables

      void initELMO();          // 
      void motorOn();           // Commands ON and zero current to start
      void motorInitOn();       // Also gives delay for initial startup
      void motorOff();          // Also command zero current right before turn off
      void motorCmd(float C);   // Commands the motor to input current, C [Amps]
      void motorCmdSat(float C, float S); // Commands motor current, with a saturation [Amps]

      uint32_t baud_;
      uint32_t recTimeout_;
    private:
      HardwareSerial &port_;
  };
}

#endif