/*
  PCBpixar.h - Library for Setup and general tasks on the Main PCB configured for communication with the Twitter

  Created by Eric R. Ambrose on January 15th, 2020
  Copyright AMBER Lab, Caltech
*/
#ifndef PCBpixar_h
#define PCBpixar_h

#include "Arduino.h"
#include "DualENC.h"
#include "SPI.h"
#include "SD.h"

class PCBpixar
{
  public:
    PCBpixar(DualENC &dENC);
    void initComm();          // Initialize all communication
    void initSD();            // Initial the onbard SD card
    void setLEDs(String val); // Set state of onboard LEDs
    int readSwitch();         // Read switch state
    void waitSwitch(int b);   // Read switch at 10Hz and return when it's ON/OFF
  private:
    DualENC &dENC_;

    int _SWCH_IN = 31;
    int _R1 = 14;
    int _Y1 = 15;
    int _G1 = 16;
    int _G2 = 17;
    int _Y2 = 18;
    int _R2 = 19;
//    int _BO4  = 33;
//    int _BO5  = 34;
//    int _BO6  = 35;
//    int _BO7  = 36;
//    int _BO8  = 37;
//    int _BO9  = 38;
//    int _BO10 = 39;
};

#endif