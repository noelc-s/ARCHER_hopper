/*
  Twitter.h - Library for sending commands to the twitter via Serial1. This version
              assumes current command mode (UM=1)

  Created by Eric R. Ambrose on August 19th, 2019
  Copyright AMBER Lab, Caltech
*/

#ifndef Twitter_h
#define Twitter_h

#include "Arduino.h"

class Twitter
{
  public:
    Twitter();                // Empty

    void initTwitter();       // 
    void motorOn();           // Also commands zero current to start
    void motorOff();          // Also command zero current right before turn off
    void motorCmd(float C);   // Commands the motor to input current, C [Amps]
};

#endif