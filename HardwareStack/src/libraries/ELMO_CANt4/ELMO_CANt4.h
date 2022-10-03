// #ifndef __ELMO_CANt4_H_INCLUDED__
// #define __ELMO_CANt4_H_INCLUDED__
#ifndef ELMO_CANt4_h
#define ELMO_CANt4_h

#include "Arduino.h"
#include "FlexCAN_T4.h"
#include "Archer_Config.h"

namespace Archer
{
  class ELMO_CANt4
  {
    public:
      enum class STATUS : uint8_t {
        OFF     = 0,
        INIT     = 1,
        MOTOR_OFF = 2,
        MOTOR_ON = 3
      };
      // enum class STATUS2 : uint8_t {
      //   OFF     = 0,
      //   INIT     = 1,
      //   MOTOR_OFF = 2,
      //   MOTOR_ON = 3
      // };
      // enum class STATUS3 : uint8_t {
      //   OFF     = 0,
      //   INIT     = 1,
      //   MOTOR_OFF = 2,
      //   MOTOR_ON = 3
      // };
      // enum class STATUS4 : uint8_t {
      //   OFF     = 0,
      //   INIT     = 1,
      //   MOTOR_OFF = 2,
      //   MOTOR_ON = 3
      // };
    public:
      ELMO_CANt4();
      int32_t initB(void);
      int32_t initK(void);
      int32_t motorOff(uint16_t IDX_);
      int32_t motorOn(uint16_t IDX_);
      int32_t setMaxC(uint16_t IDX_);
      uint32_t getMaxC(void);
      int32_t sendTC(float amps, uint16_t IDX_);
      void cmdTC(float amps, uint16_t IDX_);
      int32_t getVel(int32_t &vel, uint16_t IDX_);

      STATUS status1_;
      STATUS status2_;
      STATUS status3_;
      STATUS status4_;
      uint32_t maxC_;
      uint32_t baud_;
      int32_t  vel_;
    protected:
      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1_;
      CAN_message_t msgIn_;
      CAN_message_t msgOut_;
      const uint32_t recTimeout_;
  };
}

#endif