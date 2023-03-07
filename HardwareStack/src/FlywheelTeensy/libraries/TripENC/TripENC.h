#ifndef TripENC_H
#define TripENC_H

#include <Arduino.h>
#include <SPI.h>

class TripENC
{
  public:
    TripENC(int s1,int s2,int s3);

    void initEncoders();
    long readEncoder(int encID);				// 1 = ENC1 read, 2 = ENC2 read, 3 = ENC3 read
    void clearEncoderCount(int encID);  // 0 = All, 1 = ENC1 only, 2 = ENC2 only, 3 = ENC3 only
  private:
    int _SS1;
    int _SS2;
    int _SS3;
};

#endif