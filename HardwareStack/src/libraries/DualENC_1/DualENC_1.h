/*
  CPU.h - Library for DualENC using SPI 1 port (teensy 4.1) instead of default SPI port

  Created by Eric R. Ambrose on August 19th, 2019
  Copyright AMBER Lab, Caltech
*/

#ifndef DualENC_1_H
#define DualENC_1_H


#include <Arduino.h>
#include <SPI.h>

class DualENC_1
{
  public:
    DualENC_1(int s1,int s2);

    void initEncoders();
    long readEncoder(int encID);
    void clearEncoderCount(int encID); // 0 = Both, 1 = ENC1 only, 2 = ENC2 only
  private:
    int _SS1;
    int _SS2;
};

#endif