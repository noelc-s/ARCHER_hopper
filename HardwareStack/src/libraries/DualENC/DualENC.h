#ifndef DualENC_H
#define DualENC_H

#include <Arduino.h>
#include <SPI.h>

class DualENC
{
  public:
    DualENC(int s1, int s2);

    void initEncoders();
    long readEncoder(int encID);
    void clearEncoderCount(int encID); // 0 = Both, 1 = ENC1 only, 2 = ENC2 only
  private:
    int _SS1;
    int _SS2;
};

#endif