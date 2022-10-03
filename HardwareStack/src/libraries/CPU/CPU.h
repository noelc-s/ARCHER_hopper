/*
  CPU.h - Library for Setup and general tasks on the Main CPU configured for the Twitter

  Created by Eric R. Ambrose on August 19th, 2019
  Copyright AMBER Lab, Caltech
*/

#ifndef CPU_h
#define CPU_h

#include "Arduino.h"
#include "DualENC.h"
#include "Twitter.h"
#include "SPI.h"
#include "SD.h"
#include "vector"
#include "array"

using namespace std;

class CPU
{
  public:
    CPU(DualENC &dENC, Twitter &elmo, int A);
    ~CPU();
    void initComm();
    void initSD();
    void trajSpecs(File specFile, uint32_t &t, int &C); // Find size of data file
    void createTraj(File dataFile);                     // Create array from data file
    void loadTraj(String sFile, String hFile, uint32_t &tf, int &C0); // Reads in and creates traj array
    float findCommand(uint32_t T0);                 // Look up command for current time
    void setLEDs(String val);                       // Set state of onboard LEDs
    int readSwitch();                               // Read switch state
    void waitSwitch(int b);                         // Read switch at 10Hz and return when it's ON
    void motorStartup();                            // Wait for STO and start motor
    void resetState(int encID);                     // Reset state time
    void updateState(int encID,float &x,float&v);   // Update states to latest value
    void stateThreshold(float thresh, int a);       // Waits for state threshold crossing
    void raiseUp(float H0);                         // Reset states and wait for H0
    void delayLoop(uint32_t t0, uint32_t dtDes);    // Delays loop to given duration
    void fillStates(uint32_t t, float x1, float x2, float c);// Fills latest states into array
    void saveStates(String dFile);                          // Saves state data to SD
    void fillHop(uint32_t t, float h0, float dC, float C);  // Fills latest hop data
    void saveHop(String dFile);                             // Saves hops data to SD
    void saveHeight(String hFile, float dH);                // Saves settle height
  private:
    DualENC &dENC_;
    Twitter &elmo_;
    File data_;
    vector<pair<uint32_t, array<float, 3>>> Q_;
    vector<pair<uint32_t, array<float, 3>>> H_;

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

    uint32_t _prevT1;
    uint32_t _nextT1;
    long     _prevC1;
    long     _nextC1;
    uint32_t _prevT2;
    uint32_t _nextT2;
    long     _prevC2;
    long     _nextC2;

    float _res1 = 1024000.0;
    float _res2 = 4000.0;
    float _D2 = 0.0253;       // big whell is 0.0507, and small is 0.0253
    float _x1;
    float _x2;
    float _v1;
    float _v2;
    int _n;
    float *_traj;
};

#endif