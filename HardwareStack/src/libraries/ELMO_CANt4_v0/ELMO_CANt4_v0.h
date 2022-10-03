#ifndef __ELMO_CANT4_V0_H_INCLUDED__
#define __ELMO_CANT4_V0_H_INCLUDED__

#include "Arduino.h"
#include "FlexCAN_T4.h"
#include "Archer_Config.h"

namespace Archer
{
  class ELMO_CANt4_v0
  {
    public:
      enum class STATUS : uint8_t
      {
        OFF     = 0,
        INIT     = 1,
        MOTOR_OFF = 2,
        MOTOR_ON = 3
      };
    public:
      ELMO_CANt4_v0(uint16_t &IDX, const uint32_t &recTimeout);
      int32_t init(void);
      int32_t motorOff(void);
      int32_t motorOn(void);
      int32_t getMaxC(void);
      int32_t sendTC(float amps);
      int32_t getVel(int32_t &vel);

      STATUS status_;
      uint32_t maxC_;
      uint32_t baud_;
      int32_t  vel_;
    protected:
      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1_;
      CAN_message_t msgIn_;
      CAN_message_t msgOut_;
      const uint32_t recTimeout_;
      uint16_t IDX_;
  };
}

#endif