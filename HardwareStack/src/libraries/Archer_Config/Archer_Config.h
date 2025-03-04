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

// General_COMM_Options
#define CAN_PORT CAN1
#define CAN_BAUD 1000000
#define TTL_BAUD 1000000
#define ELMO_BAUD 115200
#define R_TIMEOUT 500000 //us

// Koios_COMM_Options
#define IDX_K1 1
#define IDX_K2 2
#define IDX_K3 3
#define K1_PORT Serial5
#define K2_PORT Serial4
#define K3_PORT Serial3
#define IMU_PORT Serial2
#define B_PORT Serial1
#define trip_CS1 10				// CS pin for Koios 1 ENC
#define trip_CS2 24				// CS pin for Koios 2 ENC
#define trip_CS3 25				// CS pin for Koios 3 ENC

// Bia_COMM_Options
#define IDX_BIA 4
#define BIA_PORT Serial3
#define K_PORT Serial2
#define doub_CS1 10				// CS pin for Bia ENC
#define doub_CS2 9				// CS pin for Foot LA11 ENC


#endif