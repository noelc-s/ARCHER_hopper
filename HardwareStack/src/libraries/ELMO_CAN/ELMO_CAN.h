#ifndef __ELMO_CAN_H_INCLUDED__
#define __ELMO_CAN_H_INCLUDED__

#include "Arduino.h"
#include "FlexCAN.h"
#include "Archer_Config.h"

namespace Archer
{
  class ELMO_CAN
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
      ELMO_CAN(FlexCAN &port, const uint16_t &IDX, const uint32_t &recTimeout);
      int32_t init(void);
      int32_t motorOff(void);
      int32_t motorOn(void);
      int32_t getMaxC(void);
      int32_t sendTC(float amps);
      int32_t getVel(void);

      STATUS status_;
      uint32_t maxC_;
      uint32_t baud_;
      int32_t  vel_;
    protected:
      FlexCAN &port_;
      CAN_message_t msgIn_;
      CAN_message_t msgOut_;
      const uint32_t recTimeout_;
      const uint16_t IDX_;
  };
}

#endif