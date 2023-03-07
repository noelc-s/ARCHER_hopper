#ifndef __ARCHER_CONFIG_H_INCLUDED__
#define __ARCHER_CONFIG_H_INCLUDED__

#include <vector>

union num32_t
{
  int32_t i;
  uint32_t ui;
  float f;
  uint8_t c[4];
};

union num16_t
{
  int16_t i;
  uint16_t ui;
  uint8_t c[2];
};

//////////////CAN_OPTIONS/////////////////
#define CAN_PORT Can0
#define CAN_BAUD 1000000
#define ELMO_FLY1_IDX 1
#define ELMO_FLY2_IDX 2
#define ELMO_FLY3_IDX 3
#define ELMO_BIA_IDX 4
#define ELMO_RECEIVE_TIMEOUT 5000 //us

#endif